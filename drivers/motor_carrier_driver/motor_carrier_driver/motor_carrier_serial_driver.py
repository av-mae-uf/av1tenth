# Date Created: March 2023
# Description: ROS2 Driver for the Arduino Nano Motor Carrier board. Will be used to serial read IMU and encoder data, while writing servo
# and drive motor commands.

import ctypes
import math
import serial
from crc import Calculator, Crc16
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class MotorCarrierDriver(Node):
    """This node communicates with the Arduino Nano and Nano Motor Carrier. It serially writes
    and receives data. The written data is the drive motor and steering servo commands and the received commands
    are the IMU data and Encoder values from each rear wheel."""

    def __init__(self):
        super().__init__("motor_carrier_driver")

        self.subscription = self.create_subscription(msg_type=AckermannDriveStamped, topic="vehicle_command_ackermann", callback=self.ackermann_control, qos_profile=1)
        self.subscription2 = self.create_subscription(msg_type=Joy, topic="joy", callback=self.joy_control, qos_profile=1)
        self.publisher = self.create_publisher(msg_type=Odometry, topic="odometry", qos_profile=1)

        self.timer1 = self.create_timer(timer_period_sec=1 / 30, callback=self.serial_read)
        self.timer2 = self.create_timer(timer_period_sec=1 / 20, callback=self.odometry_callback)
        self.arduino = serial.Serial(port="/dev/sensor/arduino", baudrate=115200)
        self.declare_parameter("Limiter", True)

        # Telling arduino the driver is active
        self.calculator = Calculator(Crc16.CCITT)
        data_bytes = bytearray([90, 90, 2, 0])
        crc16 = self.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        self.arduino.write(bytes_out)

        # Init class variables

        self.heading_degrees = 0.0
        self.encoder1_rpm = 0.0
        self.encoder2_rpm = 0.0
        self.wheel_size = 120e-3 / 2  # m
        self.axle_width = 184e-3  # m
        self.max_speed = 585 * (2 * math.pi * 60e-3) / 60
        self.flag = False

    def ackermann_control(self, msg=AckermannDriveStamped):
        """This is the callback that subscribes to the AckermannDriveStamped message and writes it to
        the serial port, along with setting the LED State"""
        limiter = self.get_parameter("Limiter").get_parameter_value().bool_value
        if self.flag is True:
            return

        if math.isfinite(msg.drive.speed) or math.isfinite(msg.drive.steering_angle) is False:
            return

        steering_angle_data = 90 - msg.drive.steering_angle * 2
        speed_data = 90 + msg.drive.speed(72 / self.max_speed)
        if limiter is True:
            speed_data = min(speed_data, 120)
            speed_data = max(speed_data, 60)
        led_color = 3  # green is 1, yellow is 2 and red is 3 with 0 off
        blink = 0
        data_bytes = bytearray([int(steering_angle_data), int(speed_data), led_color, blink])
        crc16 = self.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        self.arduino.write(bytes_out)

    def serial_read(self):
        """This callback reads the serial messages from the arduino at 30Hz to store data from the IMU and encoders."""

        try:
            if self.arduino.in_waiting:
                incoming_byte = self.arduino.read(1)
                incomingData = ord(incoming_byte)
                if incomingData == 157:
                    incoming_bytes = self.arduino.read(10)
                    if incoming_bytes[-1] == 147:
                        if (
                            self.calculator.verify(
                                incoming_bytes[0 : len(incoming_bytes) - 3],
                                (incoming_bytes[-3] << 8) | incoming_bytes[-2],
                            )
                            is True
                        ):
                            self.encoder1_rpm = ctypes.c_int16((incoming_bytes[1] << 8) | incoming_bytes[2]).value / 10.0
                            self.encoder2_rpm = ctypes.c_int16((incoming_bytes[3] << 8) | incoming_bytes[4]).value / 10.0
                            self.heading_degrees = ctypes.c_uint16((incoming_bytes[5] << 8) | incoming_bytes[6]).value / 100.0
        except Exception as ex:
            print(ex)  # Most likely the main program has closed the device or ended so just move on
            self.arduino.close()

        self.get_logger().info(f"Heading Angle (SerReadCallback): {self.heading_degrees} degrees")

    def odometry_callback(self):
        """This function publishes the odometry data at 20Hz to a topic"""

        c_z = math.cos(math.radians(self.heading_degrees) / 2)
        s_z = math.sin(math.radians(self.heading_degrees) / 2)
        c_x = math.cos(0)
        s_x = math.sin(0)
        c_y = math.cos(0)
        s_y = math.sin(0)

        left_wheel_rads = self.encoder1_rpm * ((2 * math.pi) / 60)
        right_wheel_rads = self.encoder2_rpm * ((2 * math.pi) / 60)

        msg = Odometry()
        msg.pose.pose.orientation.w = c_x * c_y * c_z + s_x * s_y * s_z
        msg.pose.pose.orientation.x = s_x * c_y * c_z - c_x * s_y * s_z
        msg.pose.pose.orientation.y = c_x * s_y * c_z + s_x * c_y * s_z
        msg.pose.pose.orientation.z = c_x * c_y * s_z - s_x * s_y * c_z

        msg.twist.twist.linear.x = (self.wheel_size / 2) * (left_wheel_rads + right_wheel_rads)
        msg.twist.twist.angular.z = (self.wheel_size / self.axle_width) * (right_wheel_rads - left_wheel_rads)

        self.get_logger().info(f"Heading Angle (OdomCallback): {self.heading_degrees} degrees")

        self.publisher.publish(msg)

    def joy_control(self, msg=Joy):
        """This Function overrides the serial write to write messages received from the Joy Node"""

        limiter = self.get_parameter("Limiter").get_parameter_value().bool_value
        self.flag = True if msg.buttons[8] == 1 else False

        if self.flag is False:
            return

        speed_data = 90 + ((-msg.axes[5] + 1) / 2 - (-msg.axes[2] + 1) / 2) * 72
        steering_angle_data = 90 - msg.axes[3] * 90

        if limiter is True:
            speed_data = min(speed_data, 120)
            speed_data = max(speed_data, 60)

        if msg.buttons[0] == 1:
            led_color = 1
        elif msg.buttons[3] == 1:
            led_color = 2
        elif msg.buttons[1] == 1:
            led_color = 3
        elif msg.buttons[2] == 1:
            led_color = 4
        else:
            led_color = 0

        if msg.buttons[7] == 1:
            blink = 1
        else:
            blink = 0

        data_bytes = bytearray([int(steering_angle_data), int(speed_data), led_color, blink])
        crc16 = self.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        self.arduino.write(bytes_out)


def main(args=None):
    rclpy.init(args=args)
    motor_carrier = MotorCarrierDriver()
    try:
        rclpy.spin(motor_carrier)

    except KeyboardInterrupt:
        motor_carrier.arduino.close()

    finally:
        motor_carrier.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
