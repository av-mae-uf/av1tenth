# Program: Motor Controller ROS2 Node
# Written by: Aditya Penumarti
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: May 2022
# Description: Python node for Pololu motor controller based upon messages published
# to a topic based upon sensor data, joystick data or other requirements. It is essentially a ROS2 node that
# writes serial lines to Polulu Maestro Mini servo controller to control servos and drive motor
# This node will take steering_angle msg and throttle_cmd msg as floats to control RC Car.
# The throttle cmd will be % effort. The throttle will be limited to ensure safe operation.
# Polulu code was written from a combination of https://github.com/FRC4564/Maestro/blob/master/maestro.py
# and Pololu Maestro Documentation

import time
from math import pi, atan
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from .ports import ports

import rclpy
import serial
import os

WHEELBASE = 313e-3  # Wheelbase of the Injora SCX10 II Chassis
MAX_LINEAR_SPEED = 585*(2*pi*120e-3)/60  # m/s
R2D = 180/pi
GEAR_RATIO = 38/15  # Gear ratio of gearbox output shaft to wheels

USER = os.environ.get('USER')
PORT = ports[USER]


class serial_cmds:
    def __init__(self, port):
        # Change according to what serial port you connected the Maestro to
        self.serial_port = port
        self.serial_usb = serial.Serial(
            self.serial_port)  # Opening the serial port
        # Setting the command variable to Pololu Protocol. First command is just an arbitrary starting byte.
        self.pol_prot_cmd = chr(0xaa) + chr(0x0c)
        # Second number is device number, default is 12

    def serial_close(self):
        self.serial_usb.close()  # Closing the serial port after completion

    def cmd_out(self, cmd):
        cmd_out_byte = self.pol_prot_cmd + cmd
        self.serial_usb.write(bytes(cmd_out_byte, 'latin-1'))

    def pos_read(self, chan):

        cmd = chr(0x10) + chr(chan)
        self.cmd_out(cmd)
        lsb = ord(self.serial_usb.read())
        msb = ord(self.serial_usb.read())

        return (msb << 8)+lsb

    def set_target(self, chan, target):
        # When setting the target/acceleration/speed the first seven bits will be considered the low bits and the last 7 bits will be considered the high bits.
        #  This is shown by the target of 1500 us X 4 = 6000 which in binary is 0101110 1110000
        # Converting from microseconds to quarter microseconds for data transmission
        # target = 1500*4 # Uncomment for debugging
        lsb = int(target) & 0x7f
        msb = (int(target) >> 7) & 0x7f
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.cmd_out(cmd)

    def set_speed(self, chan, speed):
        lsb = speed & 0x7f
        msb = (speed >> 7) & 0x7f
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.cmd_out(cmd)

    def set_acc(self, chan, acc):
        lsb = acc & 0x7f
        msb = (acc >> 7) & 0x7f
        # This is the cmd byte data for the acceleration state
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.cmd_out(cmd)


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')

        self.serial_ = serial_cmds(PORT)
        self.serial_.set_target(3, 3000)
        self.subscription = self.create_subscription(
            Twist, 'vehicle_command', self.cmd_send, 20)
        self.subscription  # prevent unused variable warning
        self.sub2 = self.create_subscription(
            Int16, 'led_color', self.led_cllbk, 20)
        self.declare_parameter('steering_offset', 0)
        self.str_offset = self.get_parameter(
            'steering_offset').get_parameter_value().double_value
        self.sub2
        self.cmd_send
        self.lastmsg = 0
        self.last_num = 0

    def led_cllbk(self, Int16):

        msg = Int16.data
        if msg == 2:
            self.serial_.set_target(3, 3000)  # Green LED
            self.serial_.set_target(5, 6000)  # Yellow LED

        elif msg == 1:
            self.serial_.set_target(4, 6000)  # Red LED
            self.serial_.set_target(3, 3000)

    def cmd_send(self, Twist):

        msg = Twist

        self.linear = float(msg.linear.x)  # m/s, +ve for fwd, -ve for rev
        self.angular = float(msg.angular.z)  # rad/s, +ve for CCW rotation

        if self.linear > 2:
            self.linear = 1.9

        # The neutral point PWM period for both the servo and the brushed motor is about 1500 us. The Maestro Servo Controller requires values
        # in quarter-microseconds,i.e. microseconds*4. The new neutral point will now be 1500*4 = 6000 quarter-microseconds.

        # The range of PWM signals for the steering servo is between 750 and 2500 microseconds, which corresponds to 3000 and 9000
        # quarter-microseconds.

        # Multiplying by 3000 to get a value between -3000 and 3000, that will be added or subtracted later from the neutral point
        ratio_throttle = (self.linear/MAX_LINEAR_SPEED)*3000

        # 6000 is the neutral point, here essentially adding or subtracting the ratio throttle to go forward and reverse.
        thrtle_eff = round(6000 + ratio_throttle)

        # The steering angle will be given between -45 and 45 degrees, with the neutral point as 0 degrees
        if self.angular < 1e-2 and self.angular > -1e-2:
            rad_curv = float(1e17)
        else:
            rad_curv = float(self.linear/self.angular)
            if rad_curv < 1e-2 and rad_curv > -1e-2:
                rad_curv = 1e-10

        steer_angle = atan(WHEELBASE/float(rad_curv)) * \
                           R2D + self.str_offset  # degrees

        # 3000/45 is the ratio to move our points between 3000 and 9000 quarter-us.
        # This ratio, when a steering angle is mutplied with it, will give us a usable value in quarter-microseconds.
        ratio_steering = 3000/45

        # Here, the neutral point is 6000, subtracting steering angle
        steering_target = round((6000 - steer_angle*ratio_steering)

        # Setting the channel number for the board where we have plugged in our servo or drive motor respectiveley.
        steering_chan=0
        drive_chan=1

        # The following commands are setting the speed and acceleration of the servo/drive. Can be changed to be slower.
        # Speed is generally based on the PWM signal and how many microseconds you wish to take to get to the target.
        # Acceleration
        speed_steering=0
        speed_drive=0
        acc_steering=0
        acc_drive=0

        self.serial_.set_speed(steering_chan, speed_steering)
        self.serial_.set_speed(drive_chan, speed_drive)
        self.serial_.set_acc(steering_chan, acc_steering)
        self.serial_.set_acc(drive_chan, acc_drive)

        # The following commands are sending the controller the targets of our throttle and steering command

        self.serial_.set_target(drive_chan, thrtle_eff)

        self.serial_.set_target(steering_chan, steering_target)

        # Loggers for Debugging

        # self.get_logger().info('Current Vehicle Throttle: "%4.2f"' % thrtle_eff)
        # self.get_logger().info('Current Vehicle Steering: "%4.2f"' % steering_target)


def main(args=None):
    rclpy.init(args=args)

    motor_controller=MotorController()
    try:
        rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        serial_cmds(PORT).set_target(1, 6000)
        serial_cmds(PORT).set_target(4, 3000)
        serial_cmds(PORT).set_target(3, 6500)
        serial_cmds(PORT).set_target(5, 3000)
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
