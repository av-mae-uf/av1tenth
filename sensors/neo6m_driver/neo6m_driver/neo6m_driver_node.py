# Program: GPS Sensor Publisher ROS2 Node
# Written by: Aditya Penumarti & Carl Crane
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: May 2022
# Description: Python node to interface with NEO6M GPS module. Uses adafruit_gps package and publishes to a topic LatLongData with a Pose msg as UTM.

import serial
import adafruit_gps

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix


class NEO6MDriver(Node):
    def __init__(self):
        super().__init__("neo6m_driver")

        uart = serial.Serial("/dev/sensor/gps", baudrate=9600, timeout=0.5)  # Opening Serial Ports
        self.gps = adafruit_gps.GPS(uart, debug=False)  # Using UART or PySerial

        self.publisher_ = self.create_publisher(msg_type=NavSatFix, topic="GPSData", qos_profile=10)

        # Two timers. One to call the GPS update. One to publish a new gps message.
        self.update_timer = self.create_timer(timer_period_sec=0.1, callback=self.update_callback)
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

        self.get_logger().info("NEO6M driver is running...")

    def update_callback(self):
        """Call the update function at a rate faster than the new data."""
        self.gps.update()

    def timer_callback(self):
        """Publishing NavSatFix GPS Data"""

        if self.gps.has_fix is False:
            self.get_logger().warn("Waiting for fix.")
            return

        msg = NavSatFix()
        msg.latitude = self.gps.latitude
        msg.longitude = self.gps.longitude
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.status.status = -1 if self.gps.has_fix is False else 0  # -1 indicates that there is no fix, 0 the opposite
        msg.status.service = 1  # For GPS service type

        self.publisher_.publish(msg)


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
