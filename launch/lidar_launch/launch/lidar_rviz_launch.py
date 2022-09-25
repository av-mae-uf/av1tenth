import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    laser_scan_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', ['/opt/ros/foxy/share/rplidar_ros/rviz/rplidar.rviz']]

    )

    ld.add_action(laser_scan_node)
    return ld
