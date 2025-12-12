#!/usr/bin/env python3
# articubot_pi_sensors/node_read_ultrasonic.py

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import hw_config as cfg


class UltrasonicNode(Node):
    """
    node_read_Ultrasonic:
    - Measures distance with HC-SR04 on Raspberry Pi GPIO.
    - Publishes sensor_msgs/Range on cfg.TOPIC_ULTRASONIC.
    """

    SPEED_OF_SOUND = 343.0  # m/s

    def __init__(self):
        super().__init__("node_read_Ultrasonic")
        cfg.check_domain_id(self.get_logger())

        self.publisher_ = self.create_publisher(Range, cfg.TOPIC_ULTRASONIC, 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cfg.ULTRASONIC_TRIG, GPIO.OUT)
        GPIO.setup(cfg.ULTRASONIC_ECHO, GPIO.IN)

        self.get_logger().info(
            f"Ultrasonic HC-SR04 TRIG={cfg.ULTRASONIC_TRIG}, "
            f"ECHO={cfg.ULTRASONIC_ECHO}"
        )

        self.timer = self.create_timer(0.1, self.measure_distance)  # 10 Hz

    def measure_distance(self):
        try:
            # Ensure trigger is low
            GPIO.output(cfg.ULTRASONIC_TRIG, False)
            time.sleep(0.0002)

            # Send 10us pulse
            GPIO.output(cfg.ULTRASONIC_TRIG, True)
            time.sleep(0.00001)
            GPIO.output(cfg.ULTRASONIC_TRIG, False)

            # Wait for echo high with timeout
            timeout = time.time() + 0.02  # 20 ms
            while GPIO.input(cfg.ULTRASONIC_ECHO) == 0:
                start = time.time()
                if time.time() > timeout:
                    self.get_logger().warn("Ultrasonic: echo did not go HIGH")
                    return

            while GPIO.input(cfg.ULTRASONIC_ECHO) == 1:
                end = time.time()
                if time.time() > timeout:
                    self.get_logger().warn("Ultrasonic: echo stuck HIGH")
                    return

            elapsed = end - start
            distance_m = (elapsed * self.SPEED_OF_SOUND) / 2.0

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = cfg.FRAME_ULTRASONIC
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.5
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = float(distance_m)

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading ultrasonic: {e}")

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup([cfg.ULTRASONIC_TRIG, cfg.ULTRASONIC_ECHO])


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
