from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import rclpy
import yaml


IMAGE_LEFT = '/left/image_rect'
LEFT_CALIB_PATH = 'EuRoC/cam_checkerboard/left.yaml'

IMAGE_RIGHT= '/right/image_rect'
RIGHT_CALIB_PATH = 'EuRoC/cam_checkerboard/right.yaml'


DISTORTION_MODEL = 'plumb_bob'
COEFFS_LEFT = [-0.285090, 0.075403, -0.000220, 0.000126, 0.000000]
INTRINSIC_LEFT = [461.25937,   0.     , 373.16481,
           0.     , 460.33945, 253.00413,
           0.     ,   0.     ,   1.     ]
RECTIFICATION_LEFT = [ 0.99988043, -0.00377949, -0.01499454,
          0.00368102,  0.99997152, -0.00658915,
          0.01501902,  0.00653317,  0.99986586]
PROJECTION_LEFT = [439.42818,   0.     , 376.25239,   0.     ,
           0.     , 439.42818, 253.53939,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]
COEFFS_RIGHT = [-0.283986, 0.074247, 0.000279, 0.000045, 0.000000]
INTRINSIC_RIGHT = [461.84918,   0.     , 364.83964,
           0.     , 460.57497, 245.71597,
           0.     ,   0.     ,   1.     ]
RECTIFICATION_RIGHT = [ 0.99998727, -0.00113856, -0.00491501,
          0.00117079,  0.99997781,  0.0065585 ,
          0.00490744, -0.00656417,  0.99996641]
PROJECTION_RIGHT = [439.42818,   0.     , 376.25239,  86.67236,
           0.     , 439.42818, 253.53939,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]

def calib_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        calib_yml = yaml.load(f, yaml.SafeLoader)
    
    intrinsic = calib_yml['camera_matrix']['data']
    coeffs = calib_yml['distortion_coefficients']['data']
    size = (calib_yml['image_width'],calib_yml['image_height'])
    stereo_rot = calib_yml['projection_matrix']['data']
    stereo_trans = None

    return intrinsic, coeffs, size, stereo_rot, stereo_trans


class Synchronized_Images_Publisher(Node):

    def __init__(self):
        super().__init__('img_sync')
        self.p_left_img = self.create_publisher(Image, 'left_sync/image', 10)
        self.p_right_img = self.create_publisher(Image, 'right_sync/image', 10)
        self.p_left_info = self.create_publisher(CameraInfo, 'left_sync/camera_info', 10)
        self.p_right_info = self.create_publisher(CameraInfo, 'right_sync/camera_info', 10)

    def stereo_sync_callback(self, left_msg, right_msg):

        self.p_left_img.publish(left_msg)
        self.p_right_img.publish(right_msg)  # TODO: change timestamp

        left_info = CameraInfo(
            left_msg.header, left_msg.height, left_msg.width, DISTORTION_MODEL,
            COEFFS_LEFT, INTRINSIC_LEFT, RECTIFICATION_LEFT, PROJECTION_LEFT)
        self.p_left_info.publish(left_info)
        right_info = CameraInfo(
            right_msg.header, right_msg.height, right_msg.width, DISTORTION_MODEL,
            COEFFS_RIGHT, INTRINSIC_RIGHT, RECTIFICATION_RIGHT, PROJECTION_RIGHT)
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