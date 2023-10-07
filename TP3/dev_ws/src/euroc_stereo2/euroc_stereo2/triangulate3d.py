import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from euroc_stereo2_interfaces.msg import FeatureMatches
import message_filters
import cv2 as cv
import numpy as np

INFO_LEFT='/left/camera_info'
INFO_RIGHT='/right/camera_info'
FEAT_MATCHES = '/matches'


class TriangulatedPointCloud3D(Node):

    def __init__(self):
        super().__init__('triangulated_pointcloud_3d')
        self.p_pointcloud2 = self.create_publisher(PointCloud, '/pointcloud', 10)

    def matches_callback(self, match_msg, left_info, right_info):
        # unflatten match message
        pts1 = np.array(match_msg.pts1).reshape((-1,2)).T
        pts2 = np.array(match_msg.pts2).reshape((-1,2)).T
        # get projection matrices
        projmat1 = np.array(left_info.p).reshape((4,3)).T
        projmat2 = np.array(right_info.p).reshape((4,3)).T
        # calculate points in homogeneous coordinates
        points4d = cv.triangulatePoints(projmat1, projmat2, pts1, pts2)
        # self.get_logger().info(f'{points4d}')

        def point4d_to_point32(p3d):  # FIXME?
            p32 = Point32()
            p32.x = float(p3d[0]) / float(p3d[3])
            p32.y = float(p3d[1]) / float(p3d[3])
            p32.z = float(p3d[2]) / float(p3d[3])
            return p32
        
        m_pcloud = PointCloud()
        m_pcloud.header = match_msg.header
        m_pcloud.points = [point4d_to_point32(r) for r in points4d]

        self.p_pointcloud2.publish(m_pcloud)


def main(args=None):
    rclpy.init(args=args)

    triangulate_node = TriangulatedPointCloud3D()

    # Creamos subscribers a las cámaras izq y der junto a los matches
    featmatches = message_filters.Subscriber(triangulate_node, FeatureMatches, FEAT_MATCHES)
    left_info = message_filters.Subscriber(triangulate_node, CameraInfo, INFO_LEFT)
    right_info = message_filters.Subscriber(triangulate_node, CameraInfo, INFO_LEFT)

    ts = message_filters.TimeSynchronizer([featmatches, left_info, right_info], 10)
    ts.registerCallback(triangulate_node.matches_callback)

    rclpy.spin(triangulate_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
