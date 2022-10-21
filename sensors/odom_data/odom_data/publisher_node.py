# Program: IMU and Encoder Odometry Publisher ROS2 Node
# Written by: Aditya Penumarti & Carl Crane
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: July 2022
# Description: Odometry data publisher that reads data from the IMU and the QTPy.

from math import pi, cos, sin, atan
import readline
import time
import rclpy
import serial
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

LINEAR_SPEED_RATIO = (2 * pi * 120e-3) / 60
R2D = 180 / pi
D2R = pi / 180


class OdomSerial:
    def __init__(self, port, baud_rate):
        self.serial_port = port
        self.serial_usb = serial.Serial(self.serial_port, baudrate=baud_rate, timeout=10)

    def serial_read(self, bytes=1):
        byte = self.serial_usb.read(bytes).hex()
        return byte

    def serial_close(self):
        self.serial_usb.close()

    def nine_axis(self):
        self.serial_usb.write(
            bytes((chr(0xFF) + chr(0xAA) + chr(0x24) + chr(0) + chr(0x00)), "utf-8")
        )

    def six_axis(self):
        self.serial_usb.write(
            bytes((chr(0xFF) + chr(0xAA) + chr(0x24) + chr(1) + chr(0x00)), "utf-8")
        )

    def serial_readline(self):
        return self.serial_usb.readline()

    def serial_in_waiting(self):
        return self.serial_usb.in_waiting


class OdomPub(Node):
    def __init__(self):
        super().__init__("odom_pub")
        self.serial_imu = OdomSerial("/dev/sensor/imu", 9600)
        self.serial_encoder = OdomSerial("/dev/sensor/encoder", 115200)
        self.publisher = self.create_publisher(Odometry, "odometry", 10)
        self.timer = self.create_timer(timer_period_sec=0.15, callback=self.odom_callback)
        self.declare_parameter("9_axis_mode", False)
        axis_mode = self.get_parameter("9_axis_mode").get_parameter_value().bool_value
        self.declare_parameter("z_angle_offset", 0.0)
        self.z_offset = self.get_parameter("z_angle_offset").get_parameter_value().double_value
        if axis_mode is True:
            self.serial_imu.nine_axis()
        else:
            self.serial_imu.six_axis()

    def odom_callback(self):

        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = 0.0
        y_acc = 0.0
        x_acc = 0.0
        z_acc = 0.0
        y_angle = 0.0
        x_angle = 0.0
        z_angle = 0.0
        mag_x = 0.0
        mag_y = 0.0
        mag_z = 0.0
        linear_speed = 0.0

        flag = False

        msg = Odometry()

        while self.serial_encoder.serial_in_waiting():
            linear_speed = float(self.serial_encoder.serial_readline()) * (2 * pi * 120e-3) / 60
        self.get_logger().info(f"Linear Speed: {linear_speed}")
        count = 0
        self.z_offset = self.get_parameter("z_angle_offset").get_parameter_value().double_value
        while self.serial_imu.serial_in_waiting():
            flag = True
            byte = self.serial_imu.serial_read()
            if byte == "55":
                outputs = self.serial_imu.serial_read()
                count += 1

                if outputs == "51" and count == 1:
                    acc_output = []
                    for x in range(9):
                        val = self.serial_imu.serial_read()
                        acc_output.append(int(val, base=16))
                    y_acc = (acc_output[1] << 8 | acc_output[0]) / 32768 * 16 * 9.81
                    x_acc = (acc_output[3] << 8 | acc_output[2]) / 32768 * 16 * 9.81
                    z_acc = (acc_output[5] << 8 | acc_output[4]) / 32768 * 16 * 9.81

                elif outputs == "52" and count > 1:
                    ang_vel_output = []
                    for x in range(9):
                        val = self.serial_imu.serial_read()
                        ang_vel_output.append(int(val, base=16))
                    pitch_rate = (ang_vel_output[1] << 8 | ang_vel_output[0]) / 32768 * 2000
                    roll_rate = (ang_vel_output[3] << 8 | ang_vel_output[2]) / 32768 * 2000
                    yaw_rate = (ang_vel_output[5] << 8 | ang_vel_output[4]) / 32768 * 2000
                elif outputs == "53" and count > 1:
                    angle_output = []
                    for x in range(9):
                        val = self.serial_imu.serial_read()
                        angle_output.append(int(val, base=16))
                    y_angle = (angle_output[1] << 8 | angle_output[0]) / 32768 * 180
                    x_angle = (angle_output[3] << 8 | angle_output[2]) / 32768 * 180
                    z_angle = (angle_output[5] << 8 | angle_output[4]) / 32768 * 180
        z_angle += self.z_offset
        c_z = cos((z_angle * D2R) / 2)
        s_z = sin((z_angle * D2R) / 2)
        c_x = cos((x_angle * D2R) / 2) * 0
        s_x = sin((x_angle * D2R) / 2) * 0
        c_y = cos((y_angle * D2R) / 2) * 0
        s_y = sin((y_angle * D2R) / 2) * 0
        msg.pose.pose.orientation.w = c_x * c_y * c_z + s_x * s_y * s_z
        msg.pose.pose.orientation.x = s_x * c_y * c_z - c_x * s_y * s_z
        msg.pose.pose.orientation.y = c_x * s_y * c_z + s_x * c_y * s_z
        msg.pose.pose.orientation.z = c_x * c_y * s_z - s_x * s_y * c_z
        msg.twist.twist.linear.x = linear_speed
        msg.twist.twist.angular.x = pitch_rate
        msg.twist.twist.angular.y = roll_rate
        msg.twist.twist.angular.z = yaw_rate

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "chassis"
        self.get_logger().info(f"yaw angle from set point {z_angle}")
        if flag is True:
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    odom_pub = OdomPub()
    try:
        rclpy.spin(odom_pub)
    except KeyboardInterrupt:
        pass
    finally:
        odom_pub.serial_encoder.serial_close()
        odom_pub.serial_imu.serial_close()
        odom_pub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
