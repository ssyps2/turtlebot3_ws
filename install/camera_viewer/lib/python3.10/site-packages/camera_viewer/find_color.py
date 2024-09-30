#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge #add dependency


class find_color(Node):
    
    def __init__(self):
        #format: .fcn() or .instance
        super().__init__("find_color") #assign node name
        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1)
        self.find_object=self.create_subscription(Image,'/image_raw/bgr_format',self.find_color_callback,image_qos_profile) #return a subscribe instance
        cv2.namedWindow('Frame')
        cv2.setMouseCallback('Frame', self.get_color)
    
    def find_color_callback(self,window:Image):
        self.frame=CvBridge().imgmsg_to_cv2(window,"bgr8")
        cv2.imshow('Frame', self.frame)
        self._user_input=cv2.waitKey(50)

    def get_user_input(self):
        return self.get_user_input
    
    # Callback function to handle mouse events
    def get_color(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Read the pixel color in BGR
            bgr_color = self.frame[y, x]
            
            # Convert BGR to RGB
            rgb_color = tuple(bgr_color[::-1])
            
            # Convert BGR to HSV
            hsv_color = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
            
            # Print the color values
            #print(f"Pixel location: ({x}, {y})")
            #print(f"RGB color: {rgb_color}")

            print(f"HSV color: {tuple(hsv_color)}")



def main():
    rclpy.init() #init routine needed for ROS2.

    find_color_node=find_color()

    while rclpy.ok():

        rclpy.spin_once(find_color_node) # Trigger callback processing.
        
        if find_color_node.get_user_input()== ord('q'):
            cv2.destroyAllWindows()
            break
    
    # Release the capture and destroy windows
    find_color_node.destroy_node()  
    rclpy.shutdown()