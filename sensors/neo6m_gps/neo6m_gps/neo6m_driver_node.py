# Program: GPS Sensor Publisher ROS2 Node
# Written by: Aditya Penumarti & Carl Crane
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: May 2022
# Description: Python node to interface with NEO6M GPS module. Uses adafruit_gps package and publishes to a topic LatLongData with a Pose msg as UTM.

import rclpy
import time
import adafruit_gps
import serial
import utm
from math import pi, sin, cos
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class NEO6MDriver(Node):
    def __init__(self):
        super().__init__("neo6m_driver")
        self.last_time_sec = 0
        uart = serial.Serial("/dev/ttyAML1", baudrate=9600, timeout=10)  # Opening Serial Ports
        self.gps = adafruit_gps.GPS(uart, debug=False)  # Using UART or PySerial
        self.gps.send_command(b"PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Set update rate to once a second (5hz) which is what you typically want.
        self.gps.send_command(b"PMTK220,200")
        self.last_print = time.monotonic()
        self.publisher_ = self.create_publisher(msg_type=Pose, topic="GPSData", qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=0.33, callback=self.timer_callback)

    def timer_callback(self):
        while self.gps.update():  # clean out buffer
            pass

        self.current = time.monotonic()
        if self.current - self.last_print >= 0.32:
            self.last_print = self.current
            while not self.gps.has_fix:
                time.sleep(0.5)
                self.get_logger().warn("Waiting for fix")
                self.gps.update()
            easting, northing, _, _ = utm.from_latlon(self.gps.latitude, self.gps.longitude)
            if self.last_time_sec != self.gps.timestamp_utc.tm_sec:
                self.last_time_sec = self.gps.timestamp_utc.tm_sec
            track_angle = self.gps.track_angle_deg

            track_angle = 999.9999 if track_angle is None else track_angle

            if track_angle is None:
                track_angle = 999.99999999
            else:
                track_angle = track_angle

            azimuth = track_angle * (pi / 180)
            msgUTM = PoseStamped()
            msgUTM.pose.position.x = easting
            msgUTM.pose.position.y = northing
            msgUTM.pose.position.z = 0.0
            msgUTM.pose.orientation.z = sin(azimuth / 2)
            msgUTM.pose.orientation.x = 0.0
            msgUTM.pose.orientation.y = 0.0
            msgUTM.pose.orientation.w = cos(azimuth / 2)
            msgUTM.header.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(msgUTM)
            self.get_logger().info(
                f"Publishing: Easting: {msgUTM.position.x}, Northing: {msgUTM.position.y} Azimuth:{azimuth}"
            )


def main(args=None):
    rclpy.init(args=args)
    neo6m_driver = NEO6MDriver()
    try:
        rclpy.spin(neo6m_driver)
    except KeyboardInterrupt:
        neo6m_driver.get_logger().warn("NEO6M Driver is Shut Down")
    finally:
        neo6m_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
