#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge  # add dependency


class FindColor(Node):
    def __init__(self):
        super().__init__("find_color")

        # Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.raw_image_subscriber = self.create_subscription(
            CompressedImage, "/image_raw/compressed", self.raw_image_callback, image_qos_profile
        )

        self.bridge = CvBridge()
        self.frame = None

        cv2.namedWindow("Frame")
        cv2.setMouseCallback("Frame", self.get_color)

    # Callback function to handle mouse events
    def get_color(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.frame is not None:
            # Read the pixel color in BGR
            bgr_color = self.frame[y, x]

            # Convert BGR to HSV
            hsv_color = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]

            # Display HSV values
            self.get_logger().info(f"HSV color: {tuple(hsv_color)}")

    def raw_image_callback(self, msg: CompressedImage):
        try:
            self.frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Frame", self.frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main():
    rclpy.init()
    find_color_node = FindColor()

    try:
        while rclpy.ok():
            rclpy.spin_once(find_color_node)  # Trigger callback processing.
    except KeyboardInterrupt:
        pass

    # Release the capture and destroy windows
    find_color_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()