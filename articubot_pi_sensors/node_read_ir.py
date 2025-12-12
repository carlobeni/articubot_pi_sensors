#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO

# ===== QoS IMPORTS (NUEVO) =====
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

import hw_config as cfg


class IRReaderNode(Node):
    def __init__(self):
        super().__init__("node_read_IR")

        cfg.check_domain_id(self.get_logger())

        # ===== QoS PARAMETERS (NUEVO) =====
        # IR → sensor rápido, no crítico
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_history", "keep_last")
        self.declare_parameter("qos_depth", 5)
        self.declare_parameter("qos_durability", "volatile")

        qos = self.build_qos_from_params()

        # ===== Publisher con QoS configurable =====
        self.publisher_ = self.create_publisher(
            Int32MultiArray,
            cfg.TOPIC_IR,
            qos
        )

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cfg.IR1_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(cfg.IR2_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.get_logger().info(
            f"IR sensors initialized on GPIO {cfg.IR1_GPIO}, {cfg.IR2_GPIO}"
        )

        # 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    # ===== QoS BUILDER (NUEVO) =====
    def build_qos_from_params(self):
        reliability = self.get_parameter("qos_reliability").value
        history = self.get_parameter("qos_history").value
        depth = self.get_parameter("qos_depth").value
        durability = self.get_parameter("qos_durability").value

        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE
            if reliability.lower() == "reliable"
            else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_ALL
            if history.lower() == "keep_all"
            else HistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
            if durability.lower() == "transient_local"
            else DurabilityPolicy.VOLATILE,
        )

    def timer_callback(self):
        try:
            ir1 = 0 if GPIO.input(cfg.IR1_GPIO) else 1
            ir2 = 0 if GPIO.input(cfg.IR2_GPIO) else 1

            msg = Int32MultiArray()
            msg.data = [ir1, ir2]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading IR sensors: {e}")

    def destroy_node(self):
        GPIO.cleanup([cfg.IR1_GPIO, cfg.IR2_GPIO])
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IRReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
