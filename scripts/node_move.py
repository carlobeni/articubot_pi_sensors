#!/usr/bin/env python3
# articubot_pi_sensors/node_move.py

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import hw_config as cfg


class MoveNode(Node):
    """
    node_move:
    - Subscribes to cfg.TOPIC_CMD_VEL_MASTER (geometry_msgs/Twist).
    - Sends movement commands via serial to Arduino Mega.
    """

    def __init__(self):
        super().__init__("node_move")
        cfg.check_domain_id(self.get_logger())

        try:
            self.ser = serial.Serial(
                cfg.MEGA_SERIAL_PORT,
                cfg.MEGA_BAUD,
                timeout=1.0
            )
            self.get_logger().info(
                f"Opened Mega serial {cfg.MEGA_SERIAL_PORT} at {cfg.MEGA_BAUD} baud"
            )
        except Exception as e:
            self.get_logger().error(f"Error opening Mega serial: {e}")
            self.ser = None

        self.sub = self.create_subscription(
            Twist,
            cfg.TOPIC_CMD_VEL_MASTER,
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        if self.ser is None:
            return

        lin = msg.linear.x
        ang = msg.angular.z

        cmd_str = f"M {lin:.3f} {ang:.3f}\n"
        try:
            self.ser.write(cmd_str.encode())
        except Exception as e:
            self.get_logger().error(f"Error writing to Mega: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
