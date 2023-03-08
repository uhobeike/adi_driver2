from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
        with_filter=LaunchConfiguration('with_filter')
        with_rviz=LaunchConfiguration('with_rviz')
        with_plot=LaunchConfiguration('with_plot')
        device=LaunchConfiguration('device')
        flame_id=LaunchConfiguration('flame_id')
        burst_read=LaunchConfiguration('burst_read')
        publish_temperature=LaunchConfiguration('publish_temperature')
        rate=LaunchConfiguration('rate')
        publish=LaunchConfiguration('publish')
        publish_debug_topic=LaunchConfiguration('publish_debug_topic')

        with_filter_launch_arg=DeclareLaunchArgument(
            'with_filter', 
            default_value='True'
        )
        with_rviz_launch_arg=DeclareLaunchArgument(
            'with_rviz', 
            default_value='False'
        )
        with_plot_launch_arg=DeclareLaunchArgument(
            'with_plot', 
            default_value='False'
        )
        device_launch_arg=DeclareLaunchArgument(
            'device', 
            default_value='dev/sensors/imu16465'
        )
        flame_id_launch_arg=DeclareLaunchArgument(
            'flame_id', 
            default_value='imu_link'
        )
        burst_read_launch_arg=DeclareLaunchArgument(
            'burst_read', 
            default_value='False'
        )
        publish_temperature_launch_arg=DeclareLaunchArgument(
            'publish_temperature', 
            default_value='False'
        )
        rate_launch_arg=DeclareLaunchArgument(
            'rate', 
            default_value='100'
        )
        publish_launch_arg=DeclareLaunchArgument(
            'publish', 
            default_value='False'
        )
        publish_debug_topic_launch_arg=DeclareLaunchArgument(
            'publish_debug_topics', 
            default_value='False'
        )
        adi_driver2_node=Node(
            package='adi_driver2', 
            namespace='adi_driver2', 
            executable='adis16465_node',
            output='screen',
            respawn='true', 
            name='adis16465_node'
        )
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('with_filter'))
        )

        return LaunchDescription([
            with_filter_launch_arg, 
            with_rviz_launch_arg, 
            with_plot_launch_arg, 
            device_launch_arg, 
            flame_id_launch_arg, 
            burst_read_launch_arg, 
            publish_temperature_launch_arg, 
            rate_launch_arg, 
            publish_launch_arg, 
            publish_debug_topic_launch_arg, 
            adi_driver2_node
        ])