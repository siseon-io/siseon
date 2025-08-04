# chj/S13P11B101/iot/ros2_ws/src/all_nodes.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='arm_control_node',
        #     executable='arm_control_node',
        #     name='arm_control_node',
        #     output='screen'
        # ),
        Node(
            package='control_bridge_node', 
            executable='control_bridge_node',
            name='control_bridge_node',
            output='screen'
        ),
        Node(
            package='eye_pos_node',
            executable='eye_pos_node', 
            name='eye_pos_node',
            output='screen'
        ),
        Node(
            package='fusion_node',
            executable='fusion_node',
            name='fusion_node',
            output='screen'
        ),
        Node(
            package='lidar_node',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen',
            parameters=[],
        ),
        Node(
            package='lidar_node',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[],
        ),
        # Node(
        #     package='manual_bt_node',
        #     executable='manual_bt_node',
        #     name='manual_bt_node',
        #     output='screen'
        # ),
        # Node(
        #     package='pairing_node',
        #     executable='pairing_node',
        #     name='pairing_node',
        #     output='screen'
        # ),
        Node(
            package='preset_bridge_node',
            executable='preset_bridge_node',
            name='preset_bridge_node',
            output='screen'
        ),
    ])