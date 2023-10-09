from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2 as cv
import message_filters
import numpy as np
import rclpy


IMAGE_LEFT = '/left/image_rect'
INFO_LEFT='/left/camera_info'
IMAGE_RIGHT= '/right/image_rect'
INFO_RIGHT='/right/camera_info'


class DenseFromDisparity(Node):

    def __init__(self):
        super().__init__('dense_from_disparity')
        self.br = CvBridge()
        self.p_disparity = self.create_publisher(Image, 'disparity', 10)
        self.p_depth = self.create_publisher(Image, 'depth', 10)

    def images_callback(self, left_msg, left_info, right_msg, right_info):
        # Transform ROS2 messages to OpenCV matrices
        left_img = self.br.imgmsg_to_cv2(left_msg, desired_encoding='passthrough')
        right_img = self.br.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')

        # Compute disparity map
        disparity_img = cv.stereoMatcher.compute(left_img, right_img)
        disparity_msg = self.br.cv2_to_imgmsg(disparity_img, encoding='passthrough')
        self.p_disparity.publish(disparity_msg)

        # Get Q matrix
        Q = None  # FIXME: compute cv.stereoRectify from camerainfo of stereo

        # Compute depth points
        # TODO: read about bitness of disparity img, it might need to be transformed
        depth_img = cv.reprojectImageTo3D(disparity_img, Q)  # 3-channel floating-point image
        depth_msg = self.br.cv2_to_imgmsg(depth_img, encoding='passthrough')
        self.p_depth.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)

    dense_node = DenseFromDisparity()

    # Create suscribers to left and right images
    left_rect = message_filters.Subscriber(dense_node, Image, IMAGE_LEFT)
    left_info = message_filters.Subscriber(dense_node, CameraInfo, IMAGE_LEFT)
    right_rect = message_filters.Subscriber(dense_node, Image, IMAGE_RIGHT)
    right_info = message_filters.Subscriber(dense_node, CameraInfo, IMAGE_RIGHT)

    ts = message_filters.TimeSynchronizer([left_rect, left_info, right_rect, right_info], 10)
    ts.registerCallback(dense_node.images_callback)

    rclpy.spin(dense_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
