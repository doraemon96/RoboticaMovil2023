import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, CameraInfo
from geometry_msgs.msg import Point32
# from euroc_stereo2_interfaces.msg import KeyPoint, KeyPointsMatched
import message_filters
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

IMAGE_LEFT = '/left/image_rect'
IMAGE_RIGHT= '/right/image_rect'
INFO_LEFT='/left/camera_info'
INFO_RIGHT='/right/camera_info'

class Features_Node(Node):

    def __init__(self):
        super().__init__('features_match')
        self.br = CvBridge()
        self.orb = cv.ORB_create()
        self.bfmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        self.p_pointcloud = self.create_publisher(PointCloud, '/keypoints/matches/good/pointcloud', 10)
        self.p_pointcloud_ransac = self.create_publisher(PointCloud, '/keypoints/matches/ransac/pointcloud', 10)
            
        # If this parameter is set to True, draw images directly into PLT
        self.declare_parameter('draw_matches', False)
        self.draw = self.get_parameter('draw_matches').value
        if self.draw:
            self.figure_all, self.ax_all = plt.subplots()
            self.ax_all.set_title('All Matches')
            self.figure_good, self.ax_good = plt.subplots()
            self.ax_good.set_title('Matches with distance < 30')
            self.figure_ransac, self.ax_ransac = plt.subplots()
            self.ax_ransac.set_title('RANSAC Matches')
            self.figure_perspective, self.ax_perspective = plt.subplots()
            self.ax_perspective.set_title('Perspective Transform')
    
        self.p_matches_img_all = self.create_publisher(Image, '/keypoints/matches/all/image', 10)
        self.p_matches_img_good = self.create_publisher(Image, '/keypoints/matches/good/image', 10)
        #self.p_matches = self.create_publisher(KeyPointsMatched, '/keypoints/matches/good/data', 10)
            
    def features_callback(self, left_img_msg, right_img_msg, left_info_msg, right_info_msg):
        left_img = self.br.imgmsg_to_cv2(left_img_msg, desired_encoding='passthrough')
        right_img = self.br.imgmsg_to_cv2(right_img_msg, desired_encoding='passthrough')
        
        #######################################################################
        # Get features and match them
        #######################################################################
        
        # Compute the keypoints and descriptors with ORB
        left_kp, left_descr = self.orb.detectAndCompute(left_img, None)
        right_kp, right_descr = self.orb.detectAndCompute(right_img, None)
        
        # Match descriptors
        matches = self.bfmatcher.match(left_descr,right_descr)

        # Good matches (i.e. distance < 30)
        good_matches = []
        for m in matches:
            if m.distance < 30:
                good_matches.append(m)

        if len(good_matches) < 1:
            return
        
        # Sorted good matches
        left_kp_good = [ left_kp[g.queryIdx].pt for g in good_matches ]
        right_kp_good = [ right_kp[g.trainIdx].pt for g in good_matches ]

        #######################################################################
        # Triangulate 3D points
        #######################################################################

        # unflatten keypoints
        pts1 = np.array(left_kp_good).reshape((-1,2)).T
        pts2 = np.array(right_kp_good).reshape((-1,2)).T

        # get projection matrices
        projmat1 = np.array(left_info_msg.p).reshape((3,4))
        projmat2 = np.array(right_info_msg.p).reshape((3,4))

        # calculate points in homogeneous coordinates
        points4d = cv.sfm.triangulatePoints(np.array([pts1, pts2]), np.array([projmat1, projmat2]))
        
        # publish PointCloud
        def point3d_to_point32(p3d):
            p32 = Point32()
            p32.x = float(p3d[0])
            p32.y = float(p3d[1])
            p32.z = float(p3d[2])
            return p32
        
        m_pcloud = PointCloud()
        m_pcloud.header = left_img_msg.header
        m_pcloud.points = [ point3d_to_point32(r) for r in points4d.T if r[2] > 0 ]

        self.p_pointcloud.publish(m_pcloud)

        #######################################################################
        # RANSAC
        #######################################################################

        # unflatten keypoints
        src_pts = np.float32(left_kp_good).reshape(-1,1,2)
        dst_pts = np.float32(right_kp_good).reshape(-1,1,2)

        # RANSAC
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        ransacMatchesMask = mask.ravel().tolist()

        ## Triangulate 3D points
        # unflatten keypoints
        pts1_ransac = np.array([left_kp_good[i] 
                                for i in range(1,len(left_kp_good)) 
                                if ransacMatchesMask[i]==1]).reshape((-1,2)).T
        pts2_ransac = np.array([right_kp_good[i] 
                                for i in range(1,len(right_kp_good)) 
                                if ransacMatchesMask[i]==1]).reshape((-1,2)).T

        # calculate points in homogeneous coordinates
        points4d = cv.sfm.triangulatePoints(np.array([pts1_ransac, pts2_ransac]), np.array([projmat1, projmat2]))
        # publish PointCloud
        m_pcloud = PointCloud()
        m_pcloud.header = left_img_msg.header
        m_pcloud.points = [ point3d_to_point32(r) for r in points4d.T if r[2] > 0 ]
        self.p_pointcloud_ransac.publish(m_pcloud)

        #######################################################################
        # Transform KeyPoints from left to right
        #######################################################################
        # transform left keypoints
        pts = pts1_ransac.T.reshape(-1,1,2)
        pts_trans = cv.perspectiveTransform(pts,M)
        # draw in right image
        kp_perspective = [ cv.KeyPoint(p[0][0], p[0][1], 1) for p in pts_trans.T ]
        img_perspective = cv.drawKeypoints(right_img, kp_perspective, None, color=(0,255,0), flags=0)
        # also draw right keypoints
        kp_ransac_right = [ cv.KeyPoint(p[0][0], p[0][1], 1) for p in pts2_ransac.reshape(-1,1,2) ]
        img_perspective = cv.drawKeypoints(img_perspective, kp_ransac_right, None, color=(0,0,255), flags=0)

        #######################################################################
        # Plot matches
        #######################################################################

        # Draw matches on images
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
            singlePointColor = None,
            matchesMask = None, 
            flags = 2)
        img_matched_all = cv.drawMatches(left_img,left_kp,right_img,right_kp,matches,None,**draw_params)
        img_matched_good = cv.drawMatches(left_img,left_kp,right_img,right_kp,good_matches,None,**draw_params)
        
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
            singlePointColor = None,
            matchesMask = ransacMatchesMask, # draw inliers
            flags = 2)
        img_matched_ransac = cv.drawMatches(left_img,left_kp,right_img,right_kp,good_matches,None,**draw_params)
        
        if self.draw:
            # Plot all matches
            if hasattr(self,'img_all'):
                self.img_all.set_data(img_matched_all)
            else:
                self.img_all = self.ax_all.imshow(img_matched_all, 'gray')
            self.figure_all.canvas.draw()
            self.figure_all.canvas.flush_events()

            # Plot good matches
            if hasattr(self,'img_good'):
                self.img_good.set_data(img_matched_good)
            else:
                self.img_good = self.ax_good.imshow(img_matched_good, 'gray')
            self.figure_good.canvas.draw()
            self.figure_good.canvas.flush_events()

            # Plot RANSAC matches
            if hasattr(self,'img_ransac'):
                self.img_ransac.set_data(img_matched_ransac)
            else:
                self.img_ransac = self.ax_ransac.imshow(img_matched_ransac, 'gray')
            self.figure_ransac.canvas.draw()
            self.figure_ransac.canvas.flush_events()

            # Plot perspective
            if hasattr(self,'img_perspective'):
                self.img_perspective.set_data(img_perspective)
            else:
                self.img_perspective = self.ax_perspective.imshow(img_perspective, 'gray')
            self.figure_perspective.canvas.draw()
            self.figure_perspective.canvas.flush_events()

        else:
            self.p_matches_img_all.publish(self.br.cv2_to_imgmsg(img_matched_all, encoding='passthrough'))
            self.p_matches_img_good.publish(self.br.cv2_to_imgmsg(img_matched_good, encoding='passthrough'))


def main(args=None):
    rclpy.init(args=args)

    plt.ion()

    features_node = Features_Node()

    # Creamos subscribers a las cámaras izq y der
    left_rect = message_filters.Subscriber(features_node, Image, IMAGE_LEFT)
    right_rect = message_filters.Subscriber(features_node, Image, IMAGE_RIGHT)
    left_info = message_filters.Subscriber(features_node, CameraInfo, INFO_LEFT)
    right_info = message_filters.Subscriber(features_node, CameraInfo, INFO_RIGHT)

    ts = message_filters.TimeSynchronizer([left_rect, right_rect, left_info, right_info], 10)
    ts.registerCallback(features_node.features_callback)

    rclpy.spin(features_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()