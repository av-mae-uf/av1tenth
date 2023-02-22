import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/sensor/lidar",
                "serial_baudrate": 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                "frame_id": "lidar",
                "inverted": False,
                "angle_compensate": True,
            }
        ],
    )    
    gps_node = Node(
        package="neo6m_driver",
        executable="driver",
        output="screen",
    )
    odometry_driver_node = Node(
        package="odometry_driver",
        executable="driver",
        output="screen",
    )
    pololu_driver_node = Node(
        package="pololu_driver",
        executable="driver",
        output="screen",
        parameters=[{"limiter" : True}],
    )

    ld.add_action(pololu_driver_node)
    ld.add_action(gps_node)
    ld.add_action(odometry_driver_node)
    ld.add_action(lidar_node)
    
    return ld
