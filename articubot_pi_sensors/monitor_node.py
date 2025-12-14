#!/usr/bin/env python3
# articubot_pi_sensors/monitor_node.py

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Range, NavSatFix, CompressedImage, Imu, MagneticField
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
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ===== DATA =====
        self.data = {}

        # ===== SUBSCRIPTIONS (SENSORS) =====
        self._sub(Int32MultiArray, cfg.TOPIC_IR, qos)
        self._sub(Range, cfg.TOPIC_ULTRASONIC, qos)
        self._sub(NavSatFix, cfg.TOPIC_GPS, qos)
        self._sub(Imu, cfg.TOPIC_IMU_GIR_ACC, qos)
        self._sub(MagneticField, cfg.TOPIC_IMU_MAG, qos)
        self._sub(CompressedImage, cfg.TOPIC_CAMERA, qos)

        # ===== SUBSCRIPTIONS (COMMANDS) =====
        self._sub(String, cfg.TOPIC_CMD_SERIAL_MEGA, qos, is_command=True)

        self.timer = self.create_timer(0.2, self.print_table)

    # --------------------------------------------------

    def _sub(self, msg_type, topic, qos, is_command=False):
        self.data[topic] = ("-", "---")

        def cb(msg):
            # ---------- COMMAND ----------
            if is_command:
                text = msg.data
                if ":" in text:
                    msg_id, value = text.split(":", 1)
                    self.data[topic] = (msg_id.strip(), value.strip())
                else:
                    self.data[topic] = ("?", text)

            # ---------- SENSOR ----------
            else:
                if hasattr(msg, "header"):
                    stamp = msg.header.stamp
                    time_str = f"{stamp.sec}.{stamp.nanosec:09d}"
                else:
                    time_str = "N/A"

                value = self._format(msg, topic)
                self.data[topic] = (time_str, value)

        self.create_subscription(msg_type, topic, cb, qos)

    # --------------------------------------------------

    def _format(self, msg, topic):

        if isinstance(msg, Int32MultiArray):
            return str(msg.data)

        if isinstance(msg, Range):
            return f"{msg.range:.3f} m"

        if isinstance(msg, NavSatFix):
            return f"{msg.latitude:.6f}, {msg.longitude:.6f}"

        # ===== IMU COMPLETA =====
        if isinstance(msg, Imu):
            q = msg.orientation
            av = msg.angular_velocity
            la = msg.linear_acceleration

            INDENT = " " * 63

            return (
                f"ori     = [{q.x:.2f}, {q.y:.2f}, {q.z:.2f}, {q.w:.2f}]\n"
                f"{INDENT}ang_vel = [{av.x:.2f}, {av.y:.2f}, {av.z:.2f}]\n"
                f"{INDENT}lin_acc = [{la.x:.2f}, {la.y:.2f}, {la.z:.2f}]"
            )


        # ===== MAGNETIC FIELD =====
        if isinstance(msg, MagneticField):
            m = msg.magnetic_field

            return f"x={m.x:.2f}, y={m.y:.2f}, z={m.z:.2f}"

        if isinstance(msg, CompressedImage):
            return "frame"

        return "?"

    # --------------------------------------------------

    def print_table(self):
        print("\033[2J\033[H", end="")
        print("=== MONITOR (PI) ===")
        print(f"ROS_DOMAIN_ID = {cfg.ROS_DOMAIN_ID}\n")

        # -------- SENSORS --------
        print("---- PUBLISHERS (SENSORS) ----")
        print("{:<35} | {:<22} | {:<70}".format("TOPIC", "TIMESTAMP", "VALUE"))
        print("-" * 135)

        for topic, (col1, value) in self.data.items():
            if topic == cfg.TOPIC_CMD_SERIAL_MEGA:
                continue
            print("{:<35} | {:<22} | {:<70}".format(topic, col1, value))

        # -------- COMMANDS --------
        print("\n---- SUBSCRIBERS (COMMANDS) ----")
        print("{:<35} | {:<22} | {:<70}".format("TOPIC", "ID", "VALUE"))
        print("-" * 135)

        if cfg.TOPIC_CMD_SERIAL_MEGA in self.data:
            msg_id, value = self.data[cfg.TOPIC_CMD_SERIAL_MEGA]
            print("{:<35} | {:<22} | {:<70}".format(
                cfg.TOPIC_CMD_SERIAL_MEGA, msg_id, value
            ))

        print("\nCtrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
