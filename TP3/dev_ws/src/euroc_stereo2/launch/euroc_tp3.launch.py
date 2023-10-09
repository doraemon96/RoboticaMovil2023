from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    return LaunchDescription([
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
                {'draw_matches': False}, # Plot matches
            ]
        )
    ])