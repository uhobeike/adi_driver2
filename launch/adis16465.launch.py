from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
        ld = LaunchDescription()

        ld.add_action(DeclareLaunchArgument('with_filter', default_value='True', description='Description of wirh_filter'))

        ld.add_action(DeclareLaunchArgument('with_rviz', default_value='False', description='Description of wirh_rviz'))

        ld.add_action(DeclareLaunchArgument('with_plot', default_value='False', description='Description of wirh_plot'))

        ld.add_action(DeclareLaunchArgument('device', default_value='dev/sensors/imu16465', description='Description of device'))
        
        ld.add_action(DeclareLaunchArgument('flame_id', default_value='imu_link', description='Description of flame_id'))

        ld.add_action(DeclareLaunchArgument('burst_read', default_value='False', description='Description of burst_read'))

        ld.add_action(DeclareLaunchArgument('publish_temperature', default_value='False', description='Description of publish_temperature'))

        ld.add_action(DeclareLaunchArgument('rate', default_value='100', description='Description of rate'))

        ld.add_action(DeclareLaunchArgument('publish', default_value='False', description='Description of publish'))

        ld.add_action(DeclareLaunchArgument('publish_debug_topic', default_value='false', description='Description of wpublish_debuf_topic'))

        ld.add_action(SetLaunchConfiguration(
                name='device', value=LaunchConfiguration('false')
        ))

        ld.add_action(Node(
            package='adi_driver2', 
            namespace='adi_driver2', 
            executable='adis16465_node',
            output='screen',
            respawn='true', 
            name='adis16465_node',
            parameters=[{'device': LaunchConfiguration('')}]
        ))

        ld.add_action(Node(
                ExecuteProcess(
                    condition=IfCondition(LaunchConfiguration('with_filter'))
                )
        ))

        return ld