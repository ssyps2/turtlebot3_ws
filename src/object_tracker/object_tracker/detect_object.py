#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge #add dependency
from geometry_msgs.msg import Twist


class detect_object(Node):
    def __init__(self):
        #format: .fcn() or .instance
        super().__init__("detect_object") #assign node name
        
        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")

        #Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
        if(self._display_image):
		# Set Up Image Viewing
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
            cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
        
        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1)
        
        self.raw_image_subscriber=self.create_subscription(
            msg_type=CompressedImage,
            topic='/image_raw/compressed',
            callback=self.raw_image_callback,
            qos_profile=image_qos_profile
        )
        self.raw_image_subscriber
        
        self.coordinate_publisher=self.create_publisher(
            msg_type=Int32MultiArray,
			topic='/object_x',
			qos_profile=image_qos_profile
		)
        self.coordinate_publisher
        
        self.image_publisher=self.create_publisher(
			msg_type=Image,
			topic='/image_raw/result',
			qos_profile=image_qos_profile
		)
        self.image_publisher

        #self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def raw_image_callback(self,ROS_frame:CompressedImage): 
       
        # Define the lower and upper bounds of the color in HSV space
        #lower_bound = np.array([60, 120, 100])   # lower bound for green
        #upper_bound = np.array([90, 170, 180])   # upper bound for green

        lower_bound = np.array([15, 140, 170])   # lower bound for yellow
        upper_bound = np.array([30, 190, 240])   # upper bound for yellow
        
        cv2_frame = CvBridge().compressed_imgmsg_to_cv2(ROS_frame, "bgr8")
        
         # Convert BGR image to HSV
        hsv = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Apply the mask to the original frame
        result = cv2.bitwise_and(cv2_frame, cv2_frame, mask=mask)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding box around the contour with the largest area
        max_area = 10
        max_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        msg=Int32MultiArray()  
        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(cv2_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            self.get_logger().info(f"Centeral Coordinate: {(x+w*.5, y+h*.5)}")       
            msg.data=[x,w]
            self.coordinate_publisher.publish(msg)
        else:
            msg.data=[int(800),int(800)] #sign for non-object
            self.coordinate_publisher.publish(msg)
            self.get_logger().info("hi")    

        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        ros_image=CvBridge().cv2_to_imgmsg(cv2_frame,"bgr8")
        self.image_publisher.publish(ros_image)
        
    def get_user_input(self):
	    return self.get_user_input


def main():
	rclpy.init() #init routine needed for ROS2.
	detect_object_node = detect_object() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(detect_object_node) # Trigger callback processing.

	#Clean up and shutdown.
	detect_object_node.destroy_node()
	rclpy.shutdown()
