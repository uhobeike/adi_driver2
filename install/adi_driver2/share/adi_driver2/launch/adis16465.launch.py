from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adi_driver2', 
            namespace='adi_driver2', 
            executable='adis16465_node',
            output='screen',
            respawn='true', 
            name='adis16465_node'
        )
    ])