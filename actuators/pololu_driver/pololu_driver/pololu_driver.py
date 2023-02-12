# Program: Pololu Driver ROS2 Node
# Written by: Aditya Penumarti
# Written For: EML4930 Autonomous Vehicles Class at the University of Florida
# Date Created: May 2022
# Description: Python driver for Pololu motor controller based upon messages published
# to a topic based upon sensor data, joystick data or other requirements. It is essentially a ROS2 node that
# writes serial lines to Polulu Maestro Mini servo controller to control servos and drive motor
# This node will take steering_angle msg and throttle_cmd msg as floats to control RC Car.
# The throttle cmd will be % effort. The throttle will be limited to ensure safe operation.
# Polulu code was written from a combination of https://github.com/FRC4564/Maestro/blob/master/maestro.py
# and Pololu Maestro Documentation

from math import pi, atan, isclose

import os
import rclpy
import serial
import time

from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from .ports import ports
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 313e-3  # Wheelbase of the Injora SCX10 II Chassis
MAX_LINEAR_SPEED = 585 * (2 * pi * 120e-3) / 60  # m/s
R2D = 180 / pi

USER = os.environ.get("USER")
PORT = ports[USER]


class SerialCmds:
    def __init__(self, port: str):
        # Change according to what serial port you connected the Maestro to
        self.serial_port = port
        self.serial_usb = serial.Serial(self.serial_port, 38400, write_timeout=0.5)  # Opening the serial port
        # Setting the command variable to Pololu Protocol. First command is just an arbitrary starting byte.
        self.pol_prot_cmd = chr(0xAA) + chr(0x0C)

        # Setting the channel number for the board where we have plugged in our servo or drive motor respectively.
        self.steering_channel = 0
        self.drive_channel = 1

        # The following commands are setting the speed and acceleration of the servo/drive. Can be changed to be slower.
        # Speed is generally based on the PWM signal and how many microseconds you wish to take to get to the target.
        # Acceleration
        self.set_speed(channel=self.steering_channel, speed=0)
        self.set_speed(channel=self.drive_channel, speed=0)
        self.set_acc(channel=self.steering_channel, acc=0)
        self.set_acc(channel=self.drive_channel, acc=0)

        self.last_throttle_effort = 0.0
        self.last_steering_target = 0.0
        self.last_light_color = "off"

    def serial_close(self) -> None:
        if self.serial_usb.is_open:
            self.serial_usb.close()  # Closing the serial port after completion

    def cmd_out(self, cmd: str) -> bool:
        cmd_out_byte = self.pol_prot_cmd + cmd
        try:
            self.serial_usb.write(bytes(cmd_out_byte, "latin-1"))
            return True
        except serial.SerialException as ex:
            # It is ok if it fails to write. Another message will come.
            return False

    def set_target(self, channel: int, target: float) -> bool:
        """
        When setting the target/acceleration/speed the first seven bits will be considered the low bits and
        the last 7 bits will be considered the high bits.
        This is shown by the target of 1500 us X 4 = 6000 which in binary is 0101110 1110000
        Converting from microseconds to quarter microseconds for data transmission
        """
        lsb, msb = int(target) & 0x7F, (int(target) >> 7) & 0x7F
        cmd = chr(0x04) + chr(channel) + chr(lsb) + chr(msb)
        successful = self.cmd_out(cmd)
        return successful

    def set_speed(self, channel: int, speed: int) -> None:
        lsb, msb = speed & 0x7F, (speed >> 7) & 0x7F
        cmd = chr(0x07) + chr(channel) + chr(lsb) + chr(msb)
        self.cmd_out(cmd)

    def set_acc(self, channel: int, acc: int) -> None:
        lsb, msb = acc & 0x7F, (acc >> 7) & 0x7F
        # This is the cmd byte data for the acceleration state
        cmd = chr(0x09) + chr(channel) + chr(lsb) + chr(msb)
        self.cmd_out(cmd)

    def update_cmds(self, throttle_effort: float, steering_target: float) -> None:
        """Use to send throttle and steering commands to the pololu"""
        # Don't send repeated commands if they have not changed.
        if not isclose(throttle_effort, self.last_throttle_effort, rel_tol=0.005):
            successful = self.set_target(channel=self.drive_channel, target=throttle_effort)
            if successful == True:  # Only save previous value if the command was successfully sent to pololu
                self.last_throttle_effort = throttle_effort

        if not isclose(steering_target, self.last_steering_target, rel_tol=0.005):
            successful = self.set_target(channel=self.steering_channel, target=steering_target)
            if successful == True:  # Only save previous value if the command was successfully sent to pololu
                self.last_steering_target = steering_target

    def set_lights(self, color: str) -> None:
        """
        Set color of LEDs. Only will send commands over serial when a different color
            is chosen than what was previously sent.
        """
        if color == "red" and self.last_light_color != "red":
            self.set_target(3, 3000)  # Green LED
            self.set_target(4, 6000)  # Red LED
            self.set_target(5, 3000)  # Yellow LED
            self.last_light_color == "red"
        elif color == "yellow" and self.last_light_color != "yellow":
            self.set_target(3, 3000)  # Green LED
            self.set_target(4, 3000)  # Red LED
            self.set_target(5, 6000)  # Yellow LED
            self.last_light_color == "yellow"
        elif color == "off" and self.last_light_color != "off":
            self.set_target(3, 3000)  # Green LED
            self.set_target(4, 3000)  # Red LED
            self.set_target(5, 3000)  # Yellow LED
            self.last_light_color == "off"

    def off_state(self) -> None:
        self.set_target(1, 6000)
        self.set_target(4, 3000)
        self.set_target(3, 6500)
        self.set_target(5, 3000)


class PololuDriver(Node):
    def __init__(self):
        self.state = "yellow"
        super().__init__("pololu_driver")

        self.pololu = SerialCmds(PORT)
        self.pololu.set_target(3, 3000)
        self.subscription1 = self.create_subscription(Twist, "vehicle_command_twist", self.twist_update, 20)
        self.publisher = self.create_publisher(Int16, "led_color",20)
        self.subscription2 = self.create_subscription(
            AckermannDriveStamped, "vehicle_command_ackermann", self.ackermann_drive_update, 20
        )
        self.timer = self.create_timer(2.0,self.led_callback)
        self.declare_parameter("steering_offset", 0.0)
        self.str_offset = self.get_parameter("steering_offset").get_parameter_value().double_value
        self.declare_parameter("limiter", True)
        self.limiter = self.get_parameter("limiter").get_parameter_value().bool_value
        self.linear_vel = 0
        self.angular_vel = 0
        self.get_logger().info("Pololu Driver Node Running")
        self.pololu.set_lights("yellow")
        

    def led_callback(self) -> None:
        """
        This callback sets the led colors on your car, based on the subscription to the topic led_color.
        """
        msg = Int16()
        if self.state == "red":
            msg.data = 1
            self.pololu.set_lights("red")
            

        elif self.state == "yellow":
            msg.data = 2
            self.pololu.set_lights("yellow")
        else:
            msg.data = 0

        self.publisher.publish(msg)
        self.state = "yellow"

    def twist_update(self, msg: Twist) -> None:
        """
        The neutral point PWM period for both the servo and the brushed motor is about 1500 us. The Maestro Servo Controller requires values
        in quarter-microseconds,i.e. microseconds*4. The new neutral point will now be 1500*4 = 6000 quarter-microseconds.

        The range of PWM signals for the steering servo is between 750 and 2500 microseconds, which corresponds to 3000 and 9000
        quarter-microseconds.
        """
        self.state = "red"        

        self.linear_vel = msg.linear.x  # m/s, +ve for fwd, -ve for rev
        self.angular_vel = msg.angular.z  # rad/s, +ve for CCW rotation
        self.str_offset = self.get_parameter("steering_offset").get_parameter_value().double_value

        if self.limiter is True:
            self.linear_vel = min(self.linear_vel, 2)
            self.linear_vel = max(self.linear_vel, -2)

        # Multiplying by 3000 to get a value between -3000 and 3000, that will be added or subtracted later from the neutral point
        ratio_throttle = (self.linear_vel / MAX_LINEAR_SPEED) * 3000

        # 6000 is the neutral point, here essentially adding or subtracting the ratio throttle to go forward and reverse.
        throttle_effort = round(6000 + ratio_throttle)

        # The steering angle will be given between -45 and 45 degrees, with the neutral point as 0 degrees
        if -1e-2 < self.angular_vel < 1e-2:
            rad_curv = float(1e17)
        else:
            rad_curv = self.linear_vel / self.angular_vel
            if -1e-2 < rad_curv < 1e-2:
                rad_curv = 1e-10

        steer_angle = atan(WHEELBASE / float(rad_curv)) * R2D + self.str_offset  # degrees

        # 3000/45 is the ratio to move our points between 3000 and 9000 quarter-us.
        # This ratio, when a steering angle is multiplied with it, will give us a usable value in quarter-microseconds.
        ratio_steering = 3000 / 45

        # Here, the neutral point is 6000, subtracting steering angle
        steering_target = round(6000 - steer_angle * ratio_steering)

        # Send new set point values to the pololu
        self.pololu.update_cmds(throttle_effort, steering_target)

    def ackermann_drive_update(self, msg: AckermannDriveStamped) -> None:
        """
        The neutral point PWM period for both the servo and the brushed motor is about 1500 us. The Maestro Servo Controller requires values
        in quarter-microseconds,i.e. microseconds*4. The new neutral point will now be 1500*4 = 6000 quarter-microseconds.

        The range of PWM signals for the steering servo is between 750 and 2500 microseconds, which corresponds to 3000 and 9000
        quarter-microseconds.
        """
        self.state = "red"
        
        self.str_offset = self.get_parameter("steering_offset").get_parameter_value().double_value
        self.throttle_effort_percentage = (msg.drive.speed/MAX_LINEAR_SPEED)*100  # m/s, +ve for fwd, -ve for rev
        self.steering_angle = msg.drive.steering_angle*R2D + self.str_offset  # deg, +ve for CCW rotation

        if self.limiter is True:
            self.throttle_effort_percentage = min(self.throttle_effort_percentage, 25.0)
            self.throttle_effort_percentage = max(self.throttle_effort_percentage, -25.0)

        # Multiplying by 3000 to get a value between -3000 and 3000, that will be added or subtracted later from the neutral point
        ratio_throttle = (self.throttle_effort_percentage / 100) * 3000

        # 6000 is the neutral point, here essentially adding or subtracting the ratio throttle to go forward and reverse.
        throttle_effort = round(6000 + ratio_throttle)

        # The steering angle will be given between -45 and 45 degrees, with the neutral point as 0 degree.
        # 3000/45 is the ratio to move our points between 3000 and 9000 quarter-us.
        # This ratio, when a steering angle is multiplied with it, will give us a usable value in quarter-microseconds.
        ratio_steering = 3000 / 45

        # Here, the neutral point is 6000, subtracting steering angle
        steering_target = round(6000 + self.steering_angle * ratio_steering)

        # Send new set point values to the pololu
        self.pololu.update_cmds(throttle_effort, steering_target)


def main(args=None):
    rclpy.init(args=args)
    msg_lights = Int16()
    msg_lights.data = 3
    pololu_driver = PololuDriver()
    try:
        rclpy.spin(pololu_driver)

    except KeyboardInterrupt:
        pololu_driver.pololu.off_state()
        pololu_driver.pololu.serial_close()
        pololu_driver.get_logger().warn("The pololu driver node is now off")

    finally:
        pololu_driver.lights.publish(msg_lights)
        time.sleep(1)
        pololu_driver.lights.publish(msg_lights)
        pololu_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
