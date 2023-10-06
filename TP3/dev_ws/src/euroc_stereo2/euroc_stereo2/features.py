import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        # self.sub_img = self.create_subscription(Image, 'left_sync/image',
        #                                       self.listener_callback, 10)
        # self.p_right_img = self.create_publisher(Image, 'right_sync/image', 10)
        # self.p_left_info = self.create_publisher(CameraInfo, 'left_sync/camera_info', 10)
        # self.p_right_info = self.create_publisher(CameraInfo, 'right_sync/camera_info', 10)
        self.br = CvBridge()
        self.orb = cv.ORB_create()
        self.bfmatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    
    def features_callback(self, left_msg, right_msg):
        left_img = self.br.imgmsg_to_cv2(left_msg)
        right_img = self.br.imgmsg_to_cv2(right_msg)
        
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
        plt.imshow(img_matched, 'gray'), plt.show() # TODO: update figure


def main(args=None):
    rclpy.init(args=args)

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