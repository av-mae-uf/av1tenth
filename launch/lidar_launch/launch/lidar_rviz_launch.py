import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    ld = LaunchDescription()

    laser_scan_view = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", [ThisLaunchFileDir(), "/pcloud_lscan_viz_config.rviz"]],
    )

    ld.add_action(laser_scan_view)
    return ld
