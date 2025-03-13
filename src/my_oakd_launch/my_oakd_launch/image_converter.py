#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageConverter(Node):
    def __init__(self):
        super().__init__("image_converter")
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "/oak/rgb/image_raw", self.image_callback, 10
        )
        self.publisher = self.create_publisher(Image, "/oak/rgb/image_raw_rgb8", 10)

        self.get_logger().info("Image Converter Node has started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convert OpenCV image back to ROS Image message
            ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
            ros_rgb_image.header = msg.header  # Preserve the timestamp and frame_id
            ros_rgb_image.encoding = "rgb8"  # Update the encoding to RGB8

            # Publish converted image
            self.publisher.publish(ros_rgb_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
