import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(package="joy", executable="joy_node", output="screen", parameters=[{"sticky_buttons": True, "autorepeat_rate": 10.0, "coalesce_interval_ms": 10}])

    motor_carrier_node = Node(package="motor_carrier_driver", executable="nano_driver", output="screen")

    ld.add_action(joy_node)
    ld.add_action(motor_carrier_node)

    return ld
