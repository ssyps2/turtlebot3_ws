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
import cv2


class Color_Track_Server(Node):
    def __init__(self):
        super().__init__('Color_Track_Server')
        self.srv = self.create_service(SetBool, 'color_track_service', self.handle_service)
        self.get_logger().info('Color Track Server ready.')
        
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

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initilize some shared parameters.
        self.cmd = Twist()
        self.cmd.angular.z = 0.0
        self.sign_x_coord = 0
        self.counter_area = 0
        self.state_status = 0 # 0 for swing mode, 1 for orientation adjust mode, 2 for job completed

    

    def handle_service(self, request, response):

        self.get_logger().info(f'Received request: {request.data}')
        self.get_logger().info("Start Color Track...")
      
      
        Timeout = False
        Timeout_time = 20.0
        running_timeStamp = self.get_clock().now().nanoseconds / 1e9

        while not Timeout and self.state_status != 2:

            if Timeout != True and self.state_status == 0:

                self.get_logger().info("Swing mode to find signs")
                time_stamp = self.get_clock().now().nanoseconds / 1e9
                current_time = self.get_clock().now().nanoseconds / 1e9
                self.cmd.angular.z = -0.25
                # self.vel_pub.publish(self.cmd)
                self.get_logger().info(f"counter area = {self.counter_area}")
                
                while self.counter_area < 2000: # Swing from -30 to 30 to find sign until the whole sign is in the pic 
                    self.get_logger().info(f"counter area = {self.counter_area}")
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if float(current_time - time_stamp) > (np.pi/6)*2 / abs(self.cmd.angular.z):
                        self.cmd.angular.z = -self.cmd.angular.z # Turn Opposite Way
                        time_stamp = self.get_clock().now().nanoseconds / 1e9 #Update time_stamp
                     
                        # self.vel_pub.publish(self.cmd)
                    if float(current_time - running_timeStamp) > Timeout_time:
                        Timeout = True
                        break
                    
                   

                self.state_status = 1


            if Timeout != True and self.state_status == 1:
                self.get_logger().info(" Orientation adjustment mode")
                while self.state_status == 1:
                    self.angle_adjustment()
                    current_time = self.get_clock().now().nanoseconds / 1e9

                    if float(current_time - running_timeStamp) > Timeout_time:
                        Timeout = True
                        # self.get_logger().info("Break! 1")
                        break
             
            
        if Timeout:
            self.get_logger().info("Timeout error!")
            self.cmd.angular.z = 0.0
            self.vel_pub.publish(self.cmd)
            response.success = False
            self.state_status = 0 # Back to inital status, since the clinent node will not shutdown when completed, the self.para will remain
            return response
        else:
            self.cmd.angular.z = 0.0
            self.vel_pub.publish(self.cmd)
            self.get_logger().info("Color Track Task completed!")
            response.success = True
            self.state_status = 0
            return response
        

    # Gain the center coords of signs based on color
    def raw_image_callback(self,ROS_frame:CompressedImage):        # The callback fcn is blocked when performancing while loops!!!

        self.get_logger().info(f"coord = {self.sign_x_coord}")
        image = CvBridge().compressed_imgmsg_to_cv2(ROS_frame, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
        # Define HSV ranges for colors
        lb_G, ub_G = np.array([50, 60, 60]), np.array([130, 180, 200])  # Green
        lb_B, ub_B = np.array([90, 40, 30]), np.array([135, 150, 140])   # Blue
        lb_R = np.array([150, 120, 90])    # lower bound for Red
        ub_R = np.array([200, 250, 280])   # upper bound for Red
        
        # Create masks for each color
        mask_G = cv2.inRange(hsv, lb_G, ub_G)
        mask_B = cv2.inRange(hsv, lb_B, ub_B)
        mask_R = cv2.inRange(hsv, lb_R, ub_R)
        
        # Combine masks
        mask = cv2.bitwise_or(cv2.bitwise_or(mask_G, mask_B), mask_R)
        
        # Apply mask to the image
        result = cv2.bitwise_and(image, image, mask=mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour
        max_area = 100
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
        
        x, y, w, h = 0, 0, image.shape[1], image.shape[0]  # Default to the entire image

        msg=Int32MultiArray()  
        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # self.get_logger().info(f"Centeral Coordinate: {(x+w*.5, y+h*.5)}")       
            msg.data=[x,w]
            self.coordinate_publisher.publish(msg)
            # Store current x coord of target
            self.sign_x_coord = x + w*0.5
        else:
            msg.data=[int(800),int(800)] #sign for non-object
            self.coordinate_publisher.publish(msg)
            # self.get_logger().info("not found")
              # Store current x coord of target
            self.sign_x_coord = 1000    


        # Publish the color labeled image
        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        ros_image=CvBridge().cv2_to_imgmsg(image,"bgr8")
        self.image_publisher.publish(ros_image)

     
        
        if max_contour is not None:
            self.counter_area = w*h
            self.get_logger().info(f"coord = {self.sign_x_coord}")
        else:
            self.get_logger().info(f"coord = {self.sign_x_coord}")
            self.counter_area = 0


    # Turn the robot to make the sign in the middle of image
    def angle_adjustment(self):

        if self.sign_x_coord != 1000 :
            
            if self.sign_x_coord>190:
                self.cmd.angular.z = -0.25
                # self.get_logger().info(" Adjusting Orientation...")    
                self.get_logger().info(f" x coord = {self.sign_x_coord}, z = {self.cmd.angular.z}")    
            elif self.sign_x_coord < 120:
                self.cmd.angular.z = 0.25
                # self.get_logger().info(" Adjusting Orientation...")    
                self.get_logger().info(f" x coord = {self.sign_x_coord}, z = {self.cmd.angular.z}")       
            else:
                self.cmd.angular.z = 0.0
                self.state_status = 2
                self.get_logger().info(" Orientation Adjustment Completed! ")

        else:
            self.cmd.angular.z = 0.0
            self.get_logger().info(" Lost Sign !")
            self.state_status = 0 # Back to swing mode for looking sign    
            
        # self.vel_pub.publish(self.cmd)


                   

    def get_user_input(self):
	    return self.get_user_input

    
def main(args=None):
    rclpy.init(args=args)
    node = Color_Track_Server()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
