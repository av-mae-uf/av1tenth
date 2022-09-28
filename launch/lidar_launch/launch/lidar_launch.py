import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    laser_scan_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/sensor/lidar",
                "serial_baudrate": 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }
        ],
    )

    ld.add_action(laser_scan_node)
    return ld
