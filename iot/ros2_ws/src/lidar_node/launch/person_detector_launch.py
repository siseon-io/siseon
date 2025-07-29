from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen',
            parameters=[],
        ),
        Node(
            package='ydlidar_ros2_driver',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[],
        )
    ])