import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from euroc_stereo2_interfaces.msg import FeatureMatches
import message_filters
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


IMAGE_LEFT = '/left/image_rect'

IMAGE_RIGHT= '/right/image_rect'

class Features_Node(Node):

    def __init__(self):
        super().__init__('features_match')
        self.br = CvBridge()
        self.orb = cv.ORB_create()
        self.bfmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        self.figure_all, self.ax_all = plt.subplots()
        self.ax_all.set_title('All Matches')
        self.figure_good, self.ax_good = plt.subplots()
        self.ax_good.set_title('Matches with distance < 30')
        self.p_matches = self.create_publisher(FeatureMatches, '/matches', 10)
            
    def features_callback(self, left_msg, right_msg):
        left_img = self.br.imgmsg_to_cv2(left_msg, desired_encoding='passthrough')
        right_img = self.br.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')
        
        # compute the descriptors with ORB
        left_kp, left_descr = self.orb.detectAndCompute(left_img, None)
        right_kp, right_descr = self.orb.detectAndCompute(right_img, None)
        
        # Match descriptors
        matches = self.bfmatcher.match(left_descr,right_descr)

        # Draw
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
            singlePointColor = None,
            matchesMask = None, # draw only inliers
            flags = 2)
        
        img_matched = cv.drawMatches(left_img,left_kp,right_img,right_kp,matches,None,**draw_params)
        
        # Draw all matches
        if hasattr(self,'img_all'):
            self.img_all.set_data(img_matched)
        else:
            self.img_all = self.ax_all.imshow(img_matched, 'gray')
            
        self.figure_all.canvas.draw()
        self.figure_all.canvas.flush_events()

        # Maintain good results
        good = []
        for m in matches:
            if m.distance < 30:
                good.append(m)
        img_matched_good = cv.drawMatches(left_img,left_kp,right_img,right_kp,good,None,**draw_params)
        
        # Draw good matches
        if hasattr(self,'img_good'):
            self.img_good.set_data(img_matched_good)
        else:
            self.img_good = self.ax_good.imshow(img_matched_good, 'gray')
            
        self.figure_good.canvas.draw()
        self.figure_good.canvas.flush_events()

        left_kpts_good = [left_kp[i] for i in [g.queryIdx for g in good]]
        right_kpts_good = [right_kp[i] for i in [g.trainIdx for g in good]]
        left_kpts_coords = [[p.pt[0], p.pt[1]] for p in left_kpts_good]
        right_kpts_coords = [[p.pt[0], p.pt[1]] for p in right_kpts_good]

        m_matches = FeatureMatches()
        m_matches.header = left_msg.header
        m_matches.pts1 = [i for s in left_kpts_coords for i in s]
        m_matches.pts2 = [i for s in right_kpts_coords for i in s]

        self.p_matches.publish(m_matches)


def main(args=None):
    rclpy.init(args=args)

    plt.ion()

    features_node = Features_Node()

    # Creamos subscribers a las cámaras izq y der
    left_rect = message_filters.Subscriber(features_node, Image, IMAGE_LEFT)
    right_rect = message_filters.Subscriber(features_node, Image, IMAGE_RIGHT)

    ts = message_filters.TimeSynchronizer([left_rect, right_rect], 10)
    ts.registerCallback(features_node.features_callback)

    rclpy.spin(features_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()