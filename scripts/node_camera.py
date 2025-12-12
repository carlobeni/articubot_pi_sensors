#!/usr/bin/env python3
# articubot_pi_sensors/node_camera.py

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import hw_config as cfg



class CameraNode(Node):
    """
    node_camera:
    - Captures frames from Raspberry Pi camera via OpenCV.
    - Publishes sensor_msgs/CompressedImage on cfg.TOPIC_CAMERA.
    """

    def __init__(self):
        super().__init__("node_camera")
        cfg.check_domain_id(self.get_logger())

        self.publisher_ = self.create_publisher(CompressedImage, cfg.TOPIC_CAMERA, 10)

        self.cap = cv2.VideoCapture(cfg.CAMERA_DEVICE_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera device")
        else:
            self.get_logger().info(f"Camera opened on index {cfg.CAMERA_DEVICE_INDEX}")

        self.timer = self.create_timer(0.1, self.capture_frame)  # 10 Hz

    def capture_frame(self):
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        # Encode as JPEG for CompressedImage
        success, encoded = cv2.imencode(".jpg", frame)
        if not success:
            self.get_logger().warn("Failed to encode frame")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
