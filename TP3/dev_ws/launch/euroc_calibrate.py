from launch import LaunchDescription
from launch_ros.actions import Node

# This launch file brings up the calibration package ready to receive EuRoC calibration
# rosbag topics.

# The rosbag has to be played manually, for that we recommend running this launch first
# and then starting the rosbag, for example:

# ros2 bag play --disable-keyboard-controls ./EuRoC/cam_checkerboard/cam_checkerboard_rosbag2


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            remappings=[
                ('right', '/cam0'),
                ('left', '/cam1'),
                ('right_camera', '/cam0'),
                ('left_camera', '/cam1'),
            ],
            parameters=[
                {"size": "7x6"},
                {"square": 0.06},
                {"background_r": 200},
                {"--no-service-check": True},
            ],
        )
    ])


# To run calibrator manually:

# ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 7x6
#  --square 0.108 right:=/cam0 left:=/cam1 right_camera:=/cam0 left_camera:=/cam1
