import ctypes
import math

import serial
from crc import Calculator, Crc16

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class MotorCarrierDriver(Node):
    """
        This node is a custom ROS 2 driver for the Arduino Nano Motor Carrier board. It will be used to exchange serial 
        data to/from the board. 
            From: IMU(Heading) and Encoder data(RPM). 
            To: Servo and Drive Motor commands.
    """

    def __init__(self):
        super().__init__("motor_carrier_driver")

        # ======= Parameters =======
        self.declare_parameter("Limiter", True)

        # ======= Timers =======
        self.serial_timer = self.create_timer(timer_period_sec=1 / 30, callback=self.serial_read_timer_callback)

        self.odom_pub_timer = self.create_timer(timer_period_sec=1 / 20, callback=self.odometry_timer_callback, 
                                                callback_group=MutuallyExclusiveCallbackGroup())

        # ======= Subscriptions =======
        self.sub_1 = self.create_subscription(msg_type=AckermannDriveStamped, topic="vehicle_command_ackermann", 
                                                     callback=self.ackermann_callback, qos_profile=1)
        
        self.sub_2 = self.create_subscription(msg_type=Joy, topic="joy", callback=self.joy_callback, qos_profile=1)
        
        # ======= Publishers =======        
        self.odom_pub = self.create_publisher(msg_type=Odometry, topic="odometry", qos_profile=1)

        # ======= Serial =======
        self.arduino = serial.Serial(port="/dev/sensor/arduino", baudrate=115200)

        self.calculator = Calculator(Crc16.CCITT)
        
        # Telling arduino the driver is active
        data_bytes = bytearray([90, 90, 2, 0])
        crc16 = self.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        self.arduino.write(bytes_out)

        # ======= Variables =======
        self.heading_degrees = 0.0
        self.encoder1_rpm = 0.0
        self.encoder2_rpm = 0.0
        self.wheel_radius = 120e-3 / 2  # m
        self.axle_width = 184e-3  # m
        self.max_speed = 585 * (2 * math.pi * 60e-3) / 60
        self.flag = False
        self.state = 32

    def ackermann_callback(self, msg: AckermannDriveStamped)-> None:
        """
            This is the callback that subscribes to the AckermannDriveStamped message and writes it to
            the serial port, along with setting the LED State
        """
        if self.flag is True:
            return

        if not math.isfinite(msg.drive.speed) or not math.isfinite(msg.drive.steering_angle):
            return
        
        steering_angle = math.degrees(msg.drive.steering_angle)

        steering_angle = min(steering_angle, 45)
        steering_angle = max(steering_angle, -45)

        steering_angle_data = 90 - steering_angle * 2
        speed_data = 90 + msg.drive.speed * (72 / self.max_speed)

        limiter = self.get_parameter("Limiter").get_parameter_value().bool_value
        if limiter is True:
            speed_data = min(speed_data, 110)
            speed_data = max(speed_data, 70)

        led_color = 3  # 0: Off, 1: Green, 2: Yellow, 3: Red, 4: All
        blink = 0 # 0: Off, 1: On

        data_bytes = bytearray([int(steering_angle_data), int(speed_data), led_color, blink])
        crc16 = self.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        self.arduino.write(bytes_out)

    def serial_read_timer_callback(self)-> None:
        """This callback reads the serial messages from the arduino at 30Hz to store data from the IMU and Encoders."""
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
                            self.state = incoming_bytes[0]

        except Exception as ex:
            print(ex)  # Most likely the main program has closed the device or ended so just move on
            self.arduino.close()

        self.arduino.reset_input_buffer()

        # Used to keep the Arduino state "Active" when neither Ackermann or Joy callbacks are triggering.
        if self.state == 32: # "Inactive"
            data_bytes = bytearray([90, 90, 2, 0])
            crc16 = self.calculator.checksum(data_bytes)
            bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
            self.arduino.write(bytes_out)

    def odometry_timer_callback(self)-> None:
        """This function publishes the odometry data at 20Hz to a topic"""
        c_z = math.cos(math.radians(self.heading_degrees) / 2)
        s_z = math.sin(math.radians(self.heading_degrees) / 2)
        c_x = math.cos(0)
        s_x = math.sin(0)
        c_y = math.cos(0)
        s_y = math.sin(0)

        left_wheel_rads = self.encoder1_rpm * ((2 * math.pi) / 60)
        right_wheel_rads = self.encoder2_rpm * ((2 * math.pi) / 60)

        # self.get_logger().info(f"Left Wheel: {self.encoder1_rpm}, \n Right Wheel: {self.encoder2_rpm}")

        msg = Odometry()
        msg.pose.pose.orientation.w = c_x * c_y * c_z + s_x * s_y * s_z
        msg.pose.pose.orientation.x = s_x * c_y * c_z - c_x * s_y * s_z
        msg.pose.pose.orientation.y = c_x * s_y * c_z + s_x * c_y * s_z
        msg.pose.pose.orientation.z = c_x * c_y * s_z - s_x * s_y * c_z
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "chassis"

        msg.twist.twist.linear.x = (self.wheel_radius / 2) * (left_wheel_rads + right_wheel_rads)

        self.odom_pub.publish(msg)

    def joy_callback(self, msg: Joy)-> None:
        """ 
            This Function overrides the AckermannDriveStamped callback's ability write serial messages 
            when the "Xbox" button is pressed on a connected controller.
        """
        self.get_logger().warn(f'Joy node should be running with sticky buttons on', once=True)

        self.flag = True if msg.buttons[8] == 1 else False
        if self.flag is False:
            return

        speed_data = 90 + ((-msg.axes[5] + 1) / 2 - (-msg.axes[2] + 1) / 2) * 72
        steering_angle_data = 90 - msg.axes[0] * 90

        limiter = self.get_parameter("Limiter").get_parameter_value().bool_value

        if limiter is True:
            speed_data = min(speed_data, 110)
            speed_data = max(speed_data, 70)
        else:
            speed_data = min(speed_data,180)
            speed_data = max(speed_data,0)

        blink = 1 # 0: Off, 1: On
        led_color = 4 # 0: Off, 1: Green, 2: Yellow, 3: Red, 4: All

        data_bytes = bytearray([int(steering_angle_data), int(speed_data), led_color, blink])
        crc16 = self.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        self.arduino.write(bytes_out)


def main(args=None):
    rclpy.init(args=args)

    motor_carrier = MotorCarrierDriver()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(motor_carrier)
        executor.spin()

    except KeyboardInterrupt:
        data_bytes = bytearray([90, 90, 1, 0])
        crc16 = motor_carrier.calculator.checksum(data_bytes)
        bytes_out = bytearray([199, data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3], (crc16 >> 8) & 0xFF, crc16 & 0xFF, 200])
        motor_carrier.arduino.write(bytes_out)
        motor_carrier.arduino.close()

    finally:
        motor_carrier.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()