#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import hw_config as cfg

class IRReaderNode(Node):
    def __init__(self):
        super().__init__("node_read_IR")
        cfg.check_domain_id(self.get_logger())

        self.publisher_ = self.create_publisher(Int32MultiArray, cfg.TOPIC_IR, 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cfg.IR1_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(cfg.IR2_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.get_logger().info(
            f"IR sensors initialized on GPIO {cfg.IR1_GPIO}, {cfg.IR2_GPIO}"
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

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
