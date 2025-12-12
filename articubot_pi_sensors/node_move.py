#!/usr/bin/env python3
# articubot_pi_sensors/node_move.py

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

import hw_config as cfg


class MoveNode(Node):
    """
    node_move:
    - Subscribes to /cmd_serial (String).
    - Sends command to Arduino Mega ONLY when ID changes.
    """

    def __init__(self):
        super().__init__("node_move")
        cfg.check_domain_id(self.get_logger())

        # ===== QoS PARAMETERS =====
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("qos_depth", 10)
        self.declare_parameter("qos_durability", "volatile")

        qos = self.build_qos_from_params()

        # ===== Serial =====
        try:
            self.ser = serial.Serial(
                cfg.MEGA_SERIAL_PORT,
                cfg.MEGA_BAUD,
                timeout=1.0
            )
            self.get_logger().info(
                f"Opened Mega serial {cfg.MEGA_SERIAL_PORT} @ {cfg.MEGA_BAUD}"
            )
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

        self.last_cmd = None
        self.cmd_id = 0

        # ===== SUBSCRIBER =====
        self.sub = self.create_subscription(
            String,
            cfg.TOPIC_CMD_SERIAL_MEGA,   # "/cmd_serial"
            self.cmd_serial_callback,
            qos
        )

    def build_qos_from_params(self):
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.get_parameter("qos_depth").value,
            durability=DurabilityPolicy.VOLATILE,
        )

    def cmd_serial_callback(self, msg: String):
        if self.ser is None:
            return

        if msg.data == self.last_cmd:
            return  # no change → no envío

        self.cmd_id += 1
        self.last_cmd = msg.data

        cmd = f"{msg.data}\n"
        try:
            self.ser.write(cmd.encode())
            self.get_logger().info(
                f"[CMD ID {self.cmd_id}] Sent to Mega: {msg.data}"
            )
        except Exception as e:
            self.get_logger().error(f"Write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
