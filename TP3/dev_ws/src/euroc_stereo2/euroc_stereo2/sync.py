from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import rclpy
import yaml


IMAGE_LEFT = '/cam0'
IMAGE_RIGHT= '/cam1'

class CalibData:
    def __init__(self, interinsic, distortion_model, distortion_coeffs,
                 rectification, projection):
        self.interinsic = interinsic
        self.distortion_model = distortion_model
        self.distortion_coeffs = distortion_coeffs
        self.rectification = rectification
        self.projection = projection

def calib_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        calib_yml = yaml.load(f, yaml.SafeLoader)
    
    calib_data = CalibData(calib_yml['camera_matrix']['data'],
                          calib_yml['distortion_model'],
                          calib_yml['distortion_coefficients']['data'],
                          calib_yml['rectification_matrix']['data'],
                          calib_yml['projection_matrix']['data'])
    return calib_data

class Synchronized_Images_Publisher(Node):

    def __init__(self):
        super().__init__('stereo_sync')
        self.declare_parameter('left_calib_path', '../EuRoC/cam_checkerboard/left.yaml')
        self.declare_parameter('right_calib_path', '../EuRoC/cam_checkerboard/right.yaml')
        self.p_left_img = self.create_publisher(Image, 'left_sync/image', 10)
        self.p_right_img = self.create_publisher(Image, 'right_sync/image', 10)
        self.p_left_info = self.create_publisher(CameraInfo, 'left_sync/camera_info', 10)
        self.p_right_info = self.create_publisher(CameraInfo, 'right_sync/camera_info', 10)
        self.left_calib = calib_from_yaml(self.get_parameter('left_calib_path').get_parameter_value().string_value)
        self.right_calib = calib_from_yaml(self.get_parameter('right_calib_path').get_parameter_value().string_value)

    def stereo_sync_callback(self, left_msg, right_msg):
        self.get_logger().info('Sync!')
        self.p_left_img.publish(left_msg)
        right_msg.header.stamp = left_msg.header.stamp 
        self.p_right_img.publish(right_msg) 

        left_info = CameraInfo()
        left_info.header = left_msg.header
        left_info.height = left_msg.height
        left_info.width = left_msg.width
        left_info.distortion_model = self.left_calib.distortion_model
        left_info.d = self.left_calib.distortion_coeffs
        left_info.k = self.left_calib.interinsic
        left_info.r = self.left_calib.rectification
        left_info.p = self.left_calib.projection
        self.p_left_info.publish(left_info)

        right_info = CameraInfo()
        right_info.header = right_msg.header
        right_info.height = right_msg.height
        right_info.width = right_msg.width
        right_info.distortion_model = self.right_calib.distortion_model
        right_info.d = self.right_calib.distortion_coeffs
        right_info.k = self.right_calib.interinsic
        right_info.r = self.right_calib.rectification
        right_info.p = self.right_calib.projection
        self.p_right_info.publish(right_info)


def main(args=None):
    rclpy.init(args=args)

    sync_publisher = Synchronized_Images_Publisher()

    # Creamos subscribers a las cámaras izq y der
    left_rect = message_filters.Subscriber(sync_publisher, Image, IMAGE_LEFT)
    right_rect = message_filters.Subscriber(sync_publisher, Image, IMAGE_RIGHT)

    ts = message_filters.TimeSynchronizer([left_rect, right_rect], 10)
    ts.registerCallback(sync_publisher.stereo_sync_callback)

    rclpy.spin(sync_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()