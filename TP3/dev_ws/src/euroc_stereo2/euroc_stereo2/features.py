import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, CameraInfo
from geometry_msgs.msg import Point32, TransformStamped, Transform, PointStamped
# import tf_conversions
import tf2_ros
# from tf.msg import tfMessage
# from euroc_stereo2_interfaces.msg import KeyPoint, KeyPointsMatched
import message_filters
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R # Para computar quaterniones

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
        self.p_cam0_traj = self.create_publisher(PointCloud, '/traj/cam0', 10) ## PARA GRAFICAR TRAYECTORIA EN RVIZ, PUBLICO UN PUNTO EN EL ORIGEN DE LA CAMARA Y LO VISUALIZO VIVO POR LARGO TIEMPO

        self.tfBroadcaster = tf2_ros.TransformBroadcaster(self)

        ## VARIABLES EN EL PASO ANTERIOR PARA OBTENER TRAYECTORIA TOTAL
        self.lastStep = {
            'hasData': False,
            'pose': Transform(),
            'homT': np.eye(4),  # TRANSFORMACION HOMOGENEA INICIAL
        }

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
        # Pose estimation
        #######################################################################
        # reshape points
        pts1_ransac = pts1_ransac.T.reshape(-1, 1, 2)
        pts2_ransac = pts2_ransac.T.reshape(-1, 1, 2)

        self.get_logger().info('projmat1: ' + str(projmat1))
        self.get_logger().info('projmat1[:, 0:3]: ' + str(projmat1[:, 0:3]))
        E, E_mask = cv.findEssentialMat(pts1_ransac, pts2_ransac, projmat1[:, 0:3],
          method=cv.RANSAC, prob=0.999, threshold=3.0)

        _, R_est, T_est, mask = cv.recoverPose(E, pts1_ransac, pts2_ransac, projmat1[:, 0:3], mask=E_mask)
        self.get_logger().info('R_est:\n' + str(R_est))
        self.get_logger().info('det(R_est): ' + str(np.linalg.det(R_est)))
        #self.get_logger().info('T_est(pre): (' + type(T_est) + ') ' + str(T_est))
        baseline = - projmat2[0, 3] / projmat2[0, 0]
        # self.get_logger().info('baseline: ' + str(baseline))
        T_est = T_est * baseline
        # self.get_logger().info('T_est: ' + str(T_est))
        # self.get_logger().info('T_est[0][0]: ' + str(T_est[0][0]))

        quat = R.from_matrix(R_est).as_quat()

        tf_msg = TransformStamped()
        # tf_msg.transform.append(TransformStamped())
        tf_msg.header = left_info_msg.header
        tf_msg.child_frame_id = 'cam1'
        tf_msg.transform.translation.x = T_est[0][0]
        tf_msg.transform.translation.y = T_est[1][0]
        tf_msg.transform.translation.z = T_est[2][0]
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.tfBroadcaster.sendTransform(tf_msg)

        #######################################################################
        # Trajectory Estimation
        #######################################################################

        if self.lastStep['hasData']:
            #######################################################################
            # Get features and match them
            #######################################################################

            # Compute the keypoints and descriptors with ORB
            # left_kp, left_descr = self.orb.detectAndCompute(left_img, None)
            # right_kp, right_descr = self.orb.detectAndCompute(right_img, None)

            # Match descriptors
            matches_traj = self.bfmatcher.match(left_descr, self.lastStep['left_descr'])

            # Good matches (i.e. distance < 30)
            good_matches_traj = []
            for m in matches_traj:
                if m.distance < 30:
                    good_matches_traj.append(m)

            if len(good_matches_traj) < 1:
                return

            # Sorted good matches
            left_kp_good_traj = [left_kp[g.queryIdx].pt for g in good_matches_traj]
            pre_kp_good_traj = [self.lastStep['left_kp'][g.trainIdx].pt for g in good_matches_traj]

            #######################################################################
            # RANSAC
            #######################################################################

            # unflatten keypoints
            src_pts_traj = np.float32(left_kp_good_traj).reshape(-1, 1, 2)
            dst_pts_traj = np.float32(pre_kp_good_traj).reshape(-1, 1, 2)

            # RANSAC
            M_traj, mask_traj = cv.findHomography(src_pts_traj, dst_pts_traj, cv.RANSAC, 5.0)
            ransacMatchesMask_traj = mask_traj.ravel().tolist()

            ## Triangulate 3D points
            # unflatten keypoints
            pts1_ransac_traj = np.array([left_kp_good_traj[i]
                                    for i in range(1, len(left_kp_good_traj))
                                    if ransacMatchesMask_traj[i] == 1]).reshape((-1, 2)).T
            pre_pts1_ransac = np.array([pre_kp_good_traj[i]
                                    for i in range(1, len(pre_kp_good_traj))
                                    if ransacMatchesMask_traj[i] == 1]).reshape((-1, 2)).T

            #######################################################################
            # Pose estimation
            #######################################################################

            pts1_ransac_traj = pts1_ransac_traj.T.reshape(-1, 1, 2)
            pre_pts1_ransac = pre_pts1_ransac.T.reshape(-1, 1, 2)

            E_traj, _ = cv.findEssentialMat(pts1_ransac_traj, pre_pts1_ransac, projmat1[:, 0:3])
            _, R_est_traj, T_est_traj, mask_traj = cv.recoverPose(E_traj, pts1_ransac_traj, pre_pts1_ransac, projmat1[:, 0:3])

            baseline_traj = 0.05
            T_est_traj = T_est_traj * baseline_traj


            # TRANSFORMACION HOMOGENEA ACTUAL
            homT_1 = np.eye(4)
            homT_1[0:3, 0:3] = R_est_traj
            homT_1[0, 3] = T_est_traj[0][0]
            homT_1[1, 3] = T_est_traj[1][0]
            homT_1[2, 3] = T_est_traj[2][0]

            self.get_logger().info('homT (preCalcs):\n' + str(self.lastStep['homT']))
            self.get_logger().info('det(R_est_traj):\n' + str(np.linalg.det(R_est_traj)))
            self.get_logger().info('homT_1:\n' + str(homT_1))
            homT = homT_1 @ self.lastStep['homT']
            RotFinal = homT[0:3, 0:3]
            quat_traj = R.from_matrix(RotFinal).as_quat()
            TrasFinal = homT[0:3, 3]

            self.get_logger().info('homT (final):\n' + str(homT))
            self.get_logger().info('TrasFinal:\n' + str(TrasFinal))

            tf_msg = TransformStamped()
            # tf_msg.transform.append(TransformStamped())
            tf_msg.header = left_info_msg.header
            tf_msg.child_frame_id = 'world'
            tf_msg.transform.translation.x = TrasFinal[0]
            tf_msg.transform.translation.y = TrasFinal[1]
            tf_msg.transform.translation.z = TrasFinal[2]
            tf_msg.transform.rotation.x = quat_traj[0]
            tf_msg.transform.rotation.y = quat_traj[1]
            tf_msg.transform.rotation.z = quat_traj[2]
            tf_msg.transform.rotation.w = quat_traj[3]
            self.tfBroadcaster.sendTransform(tf_msg)

            pt_cam0 = PointCloud()
            pt_cam0.header.stamp = left_info_msg.header.stamp
            pt_cam0.header.frame_id = 'cam0'
            pt_cam0.points = [Point32()]

            self.p_cam0_traj.publish(pt_cam0)

            self.lastStep['pose'] = tf_msg.transform
            self.lastStep['homT'] = homT

        # UPDATE LAST STEP VARIABLES
        self.lastStep['left_kp'] = left_kp
        self.lastStep['left_descr'] = left_descr

        if not self.lastStep['hasData']:
            self.lastStep['hasData'] = True


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
