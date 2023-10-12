from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32
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
        self.sm = cv.StereoBM.create()
        self.p_disparity = self.create_publisher(Image, 'depth/disparity', 10)
        self.p_depth = self.create_publisher(Image, 'depth/image', 10)
        self.p_pcloud = self.create_publisher(PointCloud, 'depth/pointcloud', 10)

    def images_callback(self, left_msg, left_info, right_msg, right_info):
        # Transform ROS2 messages to OpenCV matrices
        left_img = self.br.imgmsg_to_cv2(left_msg, desired_encoding='passthrough')
        right_img = self.br.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')

        # Compute disparity map
        disparity_img = self.sm.compute(left_img.astype(np.uint8), right_img.astype(np.uint8))
        disparity_msg = self.br.cv2_to_imgmsg(disparity_img, encoding='passthrough')
        self.p_disparity.publish(disparity_msg)

        # # Get Q matrix
        # left_k, left_d = np.array(left_info.k), np.array([left_info.d]).T
        # right_k, right_d = np.array(right_info.k), np.array([right_info.d]).T
        # img_size = (left_info.height, left_info.width)
        # # FIXME: 
        # # - R Rotation matrix from the coordinate system of the first camera to the second camera
        # # - T Translation vector from the coordinate system of the first camera to the second camera
        # R, T = np.eye(3), np.array([np.array(left_info.p).reshape(3,4)[:, -1]]).T
        # R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv.stereoRectify(
        #     left_k, left_d, right_k, right_d, img_size, R, T)

        # https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6
        cx1 = left_info.p[2]
        cx2 = right_info.p[2]
        cy = left_info.p[6]
        f = left_info.p[0]
        Tx = right_info.p[3]
        Q = np.matrix(
            [[ 1, 0, 0, -cx1 ],
             [ 0, 1, 0, -cy ],
             [ 0, 0, 0, f ],
             [ 0, 0, -1 / Tx, (cx1 - cx2) / Tx ]]
        )

        # Compute depth points
        depth_img = cv.reprojectImageTo3D(disparity_img, Q)  # 3-channel floating-point image
        depth_msg = self.br.cv2_to_imgmsg(depth_img, encoding='passthrough')
        self.p_depth.publish(depth_msg)

        # Compute depth pointcloud
        def point3d_to_point32(p3d):
            p32 = Point32()
            p32.x = float(p3d[0])
            p32.y = float(p3d[1])
            p32.z = float(p3d[2])
            return p32
        
        m_pcloud = PointCloud()
        m_pcloud.header = disparity_msg.header
        m_pcloud.points = [point3d_to_point32(r) for r in depth_img.reshape((-1,3))]

        self.p_pcloud.publish(m_pcloud)


def main(args=None):
    rclpy.init(args=args)

    dense_node = DenseFromDisparity()

    # Create suscribers to left and right images
    left_rect = message_filters.Subscriber(dense_node, Image, IMAGE_LEFT)
    left_info = message_filters.Subscriber(dense_node, CameraInfo, INFO_LEFT)
    right_rect = message_filters.Subscriber(dense_node, Image, IMAGE_RIGHT)
    right_info = message_filters.Subscriber(dense_node, CameraInfo, INFO_RIGHT)

    ts = message_filters.TimeSynchronizer([left_rect, left_info, right_rect, right_info], 10)
    ts.registerCallback(dense_node.images_callback)

    rclpy.spin(dense_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
