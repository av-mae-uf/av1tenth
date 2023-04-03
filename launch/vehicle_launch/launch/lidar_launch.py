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
        executable="gps_driver",
        output="screen",
    )
    motor_carrier_driver_node = Node(
        package="motor_carrier_driver",
        executable="nano_driver",
        output="screen",
        parameters=[{"Limiter" : True}],
    )

    ld.add_action(motor_carrier_driver_node)
    ld.add_action(gps_node)
    ld.add_action(lidar_node)

    return ld
