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
        self.color_track_client = self.create_client(SetBool, 'color_track_service')
        self.sign_recognition_client = self.create_client(SetBool, 'camera_recognition_service')

        self.get_logger().info('Service client ready.')

        self.color_track_response_last = False

    def send_color_track_request(self, data):
        
        # Create the service request
        request = SetBool.Request()
        request.data = data

        # Call the service asynchronously
        future = self.color_track_client.call_async(request)

        # Wait for the future to complete
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            self.color_track_response = future.result()
            if self.color_track_response :
                
                if self.color_track_response.success:
                    
                    if self.color_track_response != self.color_track_response_last:
                        self.color_track_response_last = self.color_track_response
                        self.get_logger().info("Color Track Completed!")
                    return True
                
                else:

                    if self.color_track_response != self.color_track_response_last:
                        self.color_track_response_last = self.color_track_response
                        self.get_logger().info("Color Tracking in progress...")
                    return False
                
            else:
                self.get_logger().error("Service response is None.")
                return False
        else:
            self.get_logger().error("Service call failed or timed out.")
            return False
    
    def send_sign_recognition_request(self, data):
        # Create the service request
        request = SetBool.Request()
        request.data = data
        # Call the service asynchronously
        future = self.sign_recognition_client.call_async(request)

        # Wait for the future to complete
        rclpy.spin_until_future_complete(self, future)
        


def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    robot_status_dict = {
                "Color_Track_and_Go": 1,
                "Sign Recognition": 2,
                }

    robot_status = robot_status_dict["Color_Track_Status"]
    robot_status_last = 0

    node.get_logger().info(f'Current Robot Status: {robot_status}')
    while rclpy.ok():
              
        # rclpy.spin_once(node)  # Process incoming callbacks # It blocks following codes, i don;t know why

        if robot_status == robot_status_dict["Color_Track_Status"]:
            
            # Ensure the service is online
            while not node.color_track_client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('Waiting for color_track_service...')

            # Send the request once and check its success
            if node.send_color_track_request(True):
                robot_status = robot_status_dict["Sign Recognition"]
                node.send_color_track_request(False) # Tell the servient that the job is completed
            
           

        
        if robot_status == robot_status_dict["Sign Recognition"]:

            # # Ensure the service is online
            while not node.sign_recognition_client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('Waiting for sign_recognition_service...')

            node.send_sign_recognition_request(True)
        
        # Ensure the info will only appear, when status change
        if robot_status != robot_status_last:
            robot_status_last = robot_status
            node.get_logger().info(f'Current Robot Status: {robot_status}')
            if robot_status == robot_status_dict["Color_Track_Status"]:
                node.get_logger().info(f'Color track mode ')
        

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
