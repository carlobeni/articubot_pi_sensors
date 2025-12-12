#!/usr/bin/env python3
# articubot_pi_sensors/monitor_node.py

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float64, String
from sensor_msgs.msg import Range, NavSatFix, CompressedImage
from geometry_msgs.msg import Vector3Stamped

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

import hw_config as cfg


class MonitorNode(Node):

    def __init__(self):
        super().__init__("monitor_node")
        cfg.check_domain_id(self.get_logger())

        # ===== QoS =====
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_durability", "volatile")

        qos = self.build_qos_from_params()

        # ===== FLAGS =====
        self.declare_parameter("monitor_cmd_serial", True)
        self.declare_parameter("monitor_ir", True)
        self.declare_parameter("monitor_ultrasonic", True)
        self.declare_parameter("monitor_gps", True)
        self.declare_parameter("monitor_imu_accel", True)
        self.declare_parameter("monitor_imu_mag", True)
        self.declare_parameter("monitor_imu_compass", True)
        self.declare_parameter("monitor_camera", True)

        # ===== DATA =====
        self.data = {}
        self.ids = {}
        self.last_value = {}

        # ===== SUBSCRIPTIONS =====
        self._sub(Int32MultiArray, cfg.TOPIC_IR, qos)
        self._sub(Range, cfg.TOPIC_ULTRASONIC, qos)
        self._sub(NavSatFix, cfg.TOPIC_GPS, qos)
        self._sub(Vector3Stamped, cfg.TOPIC_IMU_ACCEL, qos)
        self._sub(Vector3Stamped, cfg.TOPIC_IMU_MAG, qos)
        self._sub(Float64, cfg.TOPIC_IMU_COMPASS, qos)
        self._sub(CompressedImage, cfg.TOPIC_CAMERA, qos)

        if self.get_parameter("monitor_cmd_serial").value:
            self._sub(String, cfg.TOPIC_CMD_SERIAL_MEGA, qos, subscriber=True)

        self.timer = self.create_timer(0.2, self.print_table)

    def build_qos_from_params(self):
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT
            if self.get_parameter("qos_reliability").value == "best_effort"
            else ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.get_parameter("qos_depth").value,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _sub(self, msg_type, topic, qos, subscriber=False):
        self.ids[topic] = 0
        self.last_value[topic] = None

        def cb(msg):
            value = self._format(msg)
            if value != self.last_value[topic]:
                self.ids[topic] += 1
                self.last_value[topic] = value
            self.data[topic] = value

        self.create_subscription(msg_type, topic, cb, qos)

    def _format(self, msg):
        if isinstance(msg, Int32MultiArray):
            return str(msg.data)
        if isinstance(msg, Range):
            return f"{msg.range:.3f} m"
        if isinstance(msg, NavSatFix):
            return f"{msg.latitude:.6f}, {msg.longitude:.6f}"
        if isinstance(msg, Vector3Stamped):
            return f"x={msg.vector.x:.2f}, y={msg.vector.y:.2f}, z={msg.vector.z:.2f}"
        if isinstance(msg, Float64):
            return f"{msg.data:.2f}"
        if isinstance(msg, String):
            return msg.data
        if isinstance(msg, CompressedImage):
            return f"frame @ {msg.header.stamp.sec}s"
        return "?"

    def print_table(self):
        print("\033[2J\033[H", end="")
        print("=== MONITOR ===")
        print(f"ROS_DOMAIN_ID = {cfg.ROS_DOMAIN_ID}\n")

        print("---- PUBLISHERS (SENSORS) ----")
        print("{:<35} | {:<4} | {:<40}".format("TOPIC", "ID", "VALUE"))
        print("-" * 90)

        for topic in sorted(self.data):
            print("{:<35} | {:<4} | {:<40}".format(
                topic, self.ids[topic], self.data[topic])
            )

        print("\n---- SUBSCRIBERS (COMMANDS) ----")
        if cfg.TOPIC_CMD_SERIAL_MEGA in self.data:
            print("{:<35} | {:<4} | {:<40}".format(
                cfg.TOPIC_CMD_SERIAL_MEGA,
                self.ids[cfg.TOPIC_CMD_SERIAL_MEGA],
                self.data[cfg.TOPIC_CMD_SERIAL_MEGA])
            )

        print("\nCtrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
