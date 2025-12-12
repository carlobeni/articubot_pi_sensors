#!/usr/bin/env python3
# articubot_pi_sensors/monitor_node.py

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Range, NavSatFix, MagneticField, CompressedImage
from geometry_msgs.msg import Vector3Stamped
import hw_config as cfg


class MonitorNode(Node):
    """
    Monitor node:
    - Subscribes to all sensor topics and prints a live table in the terminal.
    """

    def __init__(self):
        super().__init__("monitor_node")
        cfg.check_domain_id(self.get_logger())

        self.last_ir = None
        self.last_ultra = None
        self.last_gps = None
        self.last_accel = None
        self.last_mag = None
        self.last_cam_time = None

        self.create_subscription(Int32MultiArray, cfg.TOPIC_IR, self.cb_ir, 10)
        self.create_subscription(Range, cfg.TOPIC_ULTRASONIC, self.cb_ultra, 10)
        self.create_subscription(NavSatFix, cfg.TOPIC_GPS, self.cb_gps, 10)
        self.create_subscription(Vector3Stamped, cfg.TOPIC_IMU_ACCEL, self.cb_accel, 10)
        self.create_subscription(MagneticField, cfg.TOPIC_IMU_MAG, self.cb_mag, 10)
        self.create_subscription(CompressedImage, cfg.TOPIC_CAMERA, self.cb_cam, 10)

        self.timer = self.create_timer(0.2, self.print_table)

    def cb_ir(self, msg):
        self.last_ir = msg.data

    def cb_ultra(self, msg):
        self.last_ultra = msg.range

    def cb_gps(self, msg):
        self.last_gps = (msg.latitude, msg.longitude, msg.altitude)

    def cb_accel(self, msg):
        self.last_accel = (msg.vector.x, msg.vector.y, msg.vector.z)

    def cb_mag(self, msg):
        self.last_mag = (msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z)

    def cb_cam(self, msg):
        self.last_cam_time = msg.header.stamp

    def print_table(self):
        # Clear screen and go to top-left
        print("\033[2J\033[H", end="")

        print("=== SENSOR MONITOR (articubot_pi_sensors) ===")
        print(f"ROS_DOMAIN_ID = {cfg.ROS_DOMAIN_ID}")
        print("")
        print("{:<20} | {:<50}".format("Sensor", "Value"))
        print("-" * 75)

        # IR
        ir_str = "N/A"
        if self.last_ir is not None:
            ir_str = f"[{self.last_ir[0]}, {self.last_ir[1]}]"
        print("{:<20} | {:<50}".format("IR (2x)", ir_str))

        # Ultrasonic
        ultra_str = "N/A"
        if self.last_ultra is not None:
            ultra_str = f"{self.last_ultra:.3f} m"
        print("{:<20} | {:<50}".format("Ultrasonic", ultra_str))

        # GPS
        gps_str = "N/A"
        if self.last_gps is not None:
            lat, lon, alt = self.last_gps
            if not math.isnan(lat) and not math.isnan(lon):
                gps_str = f"lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f} m"
        print("{:<20} | {:<50}".format("GPS", gps_str))

        # Accel
        accel_str = "N/A"
        if self.last_accel is not None:
            ax, ay, az = self.last_accel
            accel_str = f"ax={ax:.1f}, ay={ay:.1f}, az={az:.1f}"
        print("{:<20} | {:<50}".format("Accel (IMU)", accel_str))

        # Mag
        mag_str = "N/A"
        if self.last_mag is not None:
            mx, my, mz = self.last_mag
            mag_str = f"mx={mx:.1f}, my={my:.1f}, mz={mz:.1f}"
        print("{:<20} | {:<50}".format("Magnetometer", mag_str))

        # Camera
        cam_str = "N/A"
        if self.last_cam_time is not None:
            cam_str = f"last frame @ {self.last_cam_time.sec}s"
        print("{:<20} | {:<50}".format("Camera", cam_str))

        print("-" * 75)
        print("Use Ctrl+C to stop this monitor node.")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
