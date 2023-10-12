from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "rosbag_path", default_value=TextSubstitution(text="../EuRoC/V1_01_easy")
        ),
        DeclareLaunchArgument(
            "draw_matches", default_value='false', choices=['true', 'false'],
            description='If true, images are plotted using pyplot'
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                "--loop",
                LaunchConfiguration('rosbag_path'),
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stereo_image_proc'),
                    'launch',
                    'stereo_image_proc.launch.py'
                ])
            ])
        ),
        Node(
            package='euroc_stereo2',
            executable='sync',
            remappings=[
                ('/cam0', '/cam0/image_raw'),
                ('/cam1', '/cam1/image_raw'),
                ('left_sync/image', '/left/image_raw'),
                ('left_sync/camera_info', '/left/camera_info'),
                ('right_sync/image', '/right/image_raw'),
                ('right_sync/camera_info', '/right/camera_info'),
            ],
        ),
        Node(
            package='euroc_stereo2',
            executable='features',
            parameters=[
                {'draw_matches': LaunchConfiguration('draw_matches')},  # Publish matches
            ],
            output='screen',
        ),
        # Node(
        #     package='euroc_stereo2',
        #     executable='triangulate3d',
        #     parameters=[
        #         {'draw_matches': True}, # Plot matches
        #     ]
        # )
    ])
