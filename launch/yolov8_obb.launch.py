from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(package='ros_brics',
             executable='yolov8_obb_publisher.py',
             output='screen'),
    ])
