# Program: GPS Sensor Publisher ROS2 Node
# Written by: Aditya Penumarti & Carl Crane
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: May 2022
# Description: Python node to interface with NEO6M GPS module. Uses adafruit_gps package and publishes to a topic LatLongData with a Pose msg as UTM.

from math import pi, sin, cos

import serial
import utm
import adafruit_gps

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class NEO6MDriver(Node):
    def __init__(self):
        super().__init__("neo6m_driver")

        uart = serial.Serial("/dev/sensor/gps", baudrate=9600, timeout=0.5)  # Opening Serial Ports
        self.gps = adafruit_gps.GPS(uart, debug=False)  # Using UART or PySerial

        # No idea what this does.
        self.gps.send_command(b"PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

        # Set update rate to once a second (1 Hz) which is what you typically want.
        self.gps.send_command(b"PMTK220,200")

        self.publisher_ = self.create_publisher(
            msg_type=PoseStamped, topic="GPSData", qos_profile=10
        )

        # Two timers. One to call the GPS update. One to publish a new gps message.
        self.update_timer = self.create_timer(timer_period_sec=0.1, callback=self.update_callback)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

        self.get_logger().info("NEO6M driver is running...")
    
    def update_callback(self):
        """ Call the update function at a rate faster than the new data."""
        self.gps.update()

    def timer_callback(self):

        if self.gps.has_fix == False:
            self.get_logger().warn("Waiting for fix.")
            return
            
        easting, northing, _, _ = utm.from_latlon(self.gps.latitude, self.gps.longitude)

        track_angle = 999.9999 if self.gps.track_angle_deg is None else track_angle

        azimuth = track_angle * (pi / 180)

        msgUTM = PoseStamped()
        msgUTM.header.stamp = self.get_clock().now().to_msg()
        msgUTM.pose.position.x = easting
        msgUTM.pose.position.y = northing
        msgUTM.pose.position.z = 0.0
        msgUTM.pose.orientation.z = sin(azimuth / 2)
        msgUTM.pose.orientation.x = 0.0
        msgUTM.pose.orientation.y = 0.0
        msgUTM.pose.orientation.w = cos(azimuth / 2)

        self.publisher_.publish(msgUTM)


def main(args=None):
    rclpy.init(args=args)
    neo6m_driver = NEO6MDriver()

    try:
        rclpy.spin(neo6m_driver)

    except KeyboardInterrupt:
        neo6m_driver.gps.uart.close()
        neo6m_driver.get_logger().warn("NEO6M Driver is Shut Down")

    finally:
        neo6m_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
