import time

import serial
import adafruit_gps
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix


class NEO6MDriver(Node):
    """
        This node interfaces with NEO6M GPS module. Uses adafruit_gps package and publishes to a topic LatLongData with a Pose msg as UTM.
    """

    def __init__(self):
        super().__init__("neo6m_driver")

        # ======= Timers =======
        # Two timers. One to call the GPS update. One to publish a new gps message.
        self.update_timer = self.create_timer(timer_period_sec=0.1, callback=self.update_callback)
        self.pub_timer = self.create_timer(timer_period_sec=1.0, callback=self.pub_timer_callback)

        # ======= Publishers =======
        self.publisher_ = self.create_publisher(msg_type=NavSatFix, topic="gps", qos_profile=10)

        # ======= Serial =======
        self.uart = serial.Serial("/dev/sensor/gps", baudrate=9600, timeout=0.5)  # Opening Serial Ports
        self.gps = adafruit_gps.GPS(self.uart, debug=False)  # Using UART or PySerial

        # ======= Variables =======
        self.position_deviation = 2.5  # meters

        self.get_logger().info("NEO6M Driver Started")

    def update_callback(self)-> None:
        """
            Calls the update function at a rate faster than the new data creation
            in order to empty the data queue.
        """
        self.gps.update()

    def pub_timer_callback(self)-> None:
        """ Publishing NavSatFix GPS Data """
        msg = NavSatFix()
        if self.gps.has_fix is False:
            msg.status.status = -1
            self.publisher_.publish(msg)
            return
        
        msg.latitude = self.gps.latitude
        msg.longitude = self.gps.longitude
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.status.status = 0  # -1 indicates that there is no fix, 0 the opposite
        msg.status.service = 1  # For GPS service type

        # To publish the position covariance an array must be created
        covariance_matrix = (
            (
                np.array(
                    [
                        [(self.gps.hdop * self.position_deviation) ** 2, 0, 0],
                        [0, (self.gps.hdop * self.position_deviation) ** 2, 0],
                        [0, 0, 0],
                    ]
                )
            ).flatten()
        ).tolist()
        msg.position_covariance = covariance_matrix
        msg.position_covariance_type = 1  # 1 Means approximated from HDOP

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    neo6m_driver = NEO6MDriver()

    try:
        rclpy.spin(neo6m_driver)

    except KeyboardInterrupt:
        neo6m_driver.uart.close()

        msgNODATA = NavSatFix()
        msgNODATA.status.status = -2 # Only published for Foxglove Visualization
        neo6m_driver.publisher_.publish(msgNODATA)
        time.sleep(1)
        neo6m_driver.publisher_.publish(msgNODATA)
        neo6m_driver.get_logger().warn("NEO6M Driver is Shut Down")

    finally:
        neo6m_driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()