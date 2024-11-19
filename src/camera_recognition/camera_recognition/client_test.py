import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess

import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge #add dependency
from geometry_msgs.msg import Twist


class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = None


    def send_request(self, data):
        self.client = self.create_client(SetBool, 'color_track_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        request = SetBool.Request()
        request.data = data
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Response: {future.result().success}")
        else:
            self.get_logger().error("Service call failed.")

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()

    while True:
        user_input = input("Press Enter to call the service (type 'exit' to quit): ").strip().lower()
        if user_input == 'exit':
            break

        # Start the service node, call the service, and shut it down
        node.send_request(True)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
