# Program: GPS Sensor Publisher ROS2 Node
# Written by: Aditya Penumarti & Carl Crane
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: May 2022
# Description: Python node to interface with NEO6M GPS module. Uses adafruit_gps package and publishes to a topic LatLongData with a Pose msg as UTM.

import rclpy
import time
import busio
import sys
import adafruit_gps
import serial
import utm
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.last_time_sec = 0
        uart = serial.Serial("/dev/ttyAML1",
                             baudrate=9600,
                             timeout=10)  # Opening Serial Ports
        self.gps = adafruit_gps.GPS(
            uart,
            debug=False)  # Using UART or PySerial
        # self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Set update rate to once a second (1hz) which is what you typically want.
        # self.gps.send_command(b"PMTK220,1000")
        self.gps.send_command(b"PMTK220,200")
        self.last_print = time.monotonic()
        self.publisher_ = self.create_publisher(Pose, 'GPSData', 10)
        # timer_period = 1  # seconds
        timer_period = 0.33  # seconds  (was 0.48 sec)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.fp = open('/home/cimar/lat_long.txt', 'w')

    def timer_callback(self):
        # self.gps.update()
        while self.gps.update():  # clean out buffer
            pass
        # Every 0.33 second print out current location details if there's a fix.
        self.current = time.monotonic()
        # if self.current - self.last_print >= 1.0:
        if self.current - self.last_print >= 0.32:
            self.last_print = self.current
            while not self.gps.has_fix:
                time.sleep(0.5)
            # Try again if we don't have a fix yet.
                self.get_logger().info('Waiting for fix')
                self.gps.update()
            utmData = utm.from_latlon(self.gps.latitude, self.gps.longitude)
            if (self.last_time_sec != self.gps.timestamp_utc.tm_sec):
                #               print(self.gps.timestamp_utc.tm_year, self.gps.timestamp_utc.tm_mon, self.gps.timestamp_utc.tm_mday, self.gps.timestamp_utc.tm_hour, self.gps.timestamp_utc.tm_min, self.gps.timestamp_utc.tm_sec, self.gps.latitude, self.gps.longitude, utmData[0], utmData[1], self.gps.horizontal_dilution , file=self.fp)
                self.last_time_sec = self.gps.timestamp_utc.tm_sec
            self.east = utmData[0]
            self.north = utmData[1]
            # Information for zone and letter available, but not being published at the moment
            self.zoneNum = utmData[2]
            self.zoneLetter = utmData[3]
            pi_ = math.pi
            track_angle = self.gps.track_angle_deg
            if track_angle is None:
                track_angle = 999.99999999
            else:
                track_angle = track_angle
            self.azimuth = track_angle*(pi_/180)
            msgUTM = Pose()
            msgUTM.position.x = self.east
            msgUTM.position.y = self.north
            msgUTM.position.z = 0.0
            msgUTM.orientation.z = math.sin(self.azimuth/2)
            msgUTM.orientation.x = 0.0
            msgUTM.orientation.y = 0.0
            msgUTM.orientation.w = math.cos(self.azimuth/2)
            self.publisher_.publish(msgUTM)
            self.get_logger().info(
                'Publishing: East: ' + str(msgUTM.position.x) + ' , North: ' + str(msgUTM.position.y) +
                'Azimuth:'+str(self.azimuth))
            # Forcing terminal print by converting floats to strings


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
