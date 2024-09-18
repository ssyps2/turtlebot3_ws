#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge #add dependency


class find_object(Node):
    def __init__(self):
        #format: .fcn() or .instance
        super().__init__("find_object") #assign node name
        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1)
        self.find_object=self.create_subscription(Image,'/image_raw/bgr_format',self.find_object_callback,image_qos_profile) #return a subscribe instance
        
        self.find_object_publisher=self.create_publisher(
				msg_type=Image,
				topic='/image_raw/result',
				qos_profile=image_qos_profile
		)
        self.find_object_publisher

        self.coordinate_publisher=self.create_publisher(Int32,'/object_x',image_qos_profile)

        
		

    

    def find_object_callback(self,ROS_frame:Image): 
        
       
        # Define the lower and upper bounds of the color in HSV space
        lower_bound = np.array([60, 120, 100])   # lower bound for green
        upper_bound = np.array([90, 170, 180])   # upper bound for green
        
        
        frame = CvBridge().imgmsg_to_cv2(ROS_frame, "bgr8")
         # Convert BGR image to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Apply the mask to the original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours in the mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding box around the contour with the largest area
        max_area = 100
        max_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            self.get_logger().info(f"Centeral Coordinate: {(x+w*.5, y+h*.5)}")
            msg=Int32()
            msg.data=x
            self.coordinate_publisher.publish(msg)
           

        # Display the original frame and the result
        #cv2.imshow('Frame', frame)
        #cv2.imshow('Mask', mask)
        #cv2.imshow('Result', result)

        #publish the result image
        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        ros_image=CvBridge().cv2_to_imgmsg(frame,"bgr8")
        self.find_object_publisher.publish(ros_image)

    def get_user_input(self):
	    return self.get_user_input
    







def main():
	rclpy.init() #init routine needed for ROS2.
	find_object_node = find_object() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(find_object_node) # Trigger callback processing.
		if find_object_node.get_user_input()== ord('q'):
			cv2.destroyAllWindows()
			break

	#Clean up and shutdown.
	find_object_node.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
