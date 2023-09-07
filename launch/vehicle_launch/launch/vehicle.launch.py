from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    gps_node = Node(
        package="neo6m_driver",
        executable="gps_driver",
        output="screen",
    )
    ld.add_action(gps_node)

    motor_carrier_driver_node = Node(
        package="motor_carrier_driver",
        executable="nano_driver",
        output="screen",
        parameters=[{"Limiter": True}],
    )
    ld.add_action(motor_carrier_driver_node)

    return ld