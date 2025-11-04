from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onrobot_rg_driver',
            executable='driver_node.py',
            name='rg',
            output='screen',
            parameters=[{
                'ip_address': '192.168.1.1',
                'port': 502,
                'gripper': 'rg6',
                'status_rate_hz': 20.0,
            }]
        )
    ])