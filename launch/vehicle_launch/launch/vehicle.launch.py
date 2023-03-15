import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
     
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
    
    return ld
