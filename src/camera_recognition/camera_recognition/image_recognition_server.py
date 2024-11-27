#!/usr/bin/env python3
import cv2
import csv
import math
import numpy as np
import random
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge #add dependency
from geometry_msgs.msg import Twist
import cv2


class KNNClassifier:
    def __init__(self, imageDirectory='./2024F_imgs/', imageType='.png'):
        self.imageDirectory = imageDirectory
        self.imageType = imageType

        with open(imageDirectory + 'labels.txt', 'r') as f:
            self.reader = csv.reader(f)
            self.lines = list(self.reader)

        self.train_lines = self.lines[:math.floor(len(self.lines))][:]

        self.x, self.y, self.w, self.h= [0,0,0,0]


    ## Edge enhencement to original image
    def edg_enhence(self, original_image:np.ndarray) -> np.ndarray:
        x, y, channel = np.shape(original_image) 
        enhenced_image = np.zeros([x,y,channel],dtype=np.uint8)

        for i in range(channel):
            channel_image = original_image[:,:,i]
            # Step 3: Apply Gaussian Blur to reduce noise
            channel_image = cv2.GaussianBlur(channel_image, (5, 5), 1.4)
            # Step 4: Apply Sobel filters to compute gradients
            sobel_x = cv2.Sobel(channel_image, cv2.CV_64F, 1, 0, ksize=3,scale=3)  # Gradient in x
            sobel_y = cv2.Sobel(channel_image, cv2.CV_64F, 0, 1, ksize=3,scale=3)  # Gradient in y
            
            # Compute the gradient magnitude
            sobel_magnitude = cv2.magnitude(sobel_x, sobel_y)
            sobel_magnitude = cv2.convertScaleAbs(sobel_magnitude)  # Convert to uint8 for visualization

            # Step 4: Compute the gradient magnitude (optional)
            sobel_magnitude = cv2.magnitude(sobel_x, sobel_y)

            # Convert gradients to uint8 for display purposes
            sobel_x = cv2.convertScaleAbs(sobel_x)
            sobel_y = cv2.convertScaleAbs(sobel_y)
            sobel_combined = cv2.addWeighted(sobel_x, 0.5, sobel_y, 0.5, 0)
            enhenced_image[:,:,i] = channel_image + sobel_combined

        return enhenced_image
    

    ## Enhence x gradient to tell the left and right arrow
    def x_enhencement(self, origin_image:np.ndarray):
        # Apply Sobel filter to detect x-direction gradients
        sobel_x = cv2.Sobel(origin_image, cv2.CV_64F, 1, 0, ksize=3)  # Gradient in x-direction
        sobel_x = np.absolute(sobel_x) * 3  # Take the absolute value to remove negative gradients
        sobel_x = cv2.convertScaleAbs(sobel_x)   # Convert back to uint8


        # Normalize the result to enhance differences
        sobel_x_normalized = cv2.normalize(sobel_x, None, 0, 255, cv2.NORM_MINMAX)
        combined_image = cv2.addWeighted(origin_image, 0.7, sobel_x_normalized, 0.3, 0)
        # combined_image = cv2.GaussianBlur(combined_image, (5, 5), 1.4)
        return combined_image


    ## Contours processing
    def process_contours(self, image, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area in descending order
        min_contour_area = 200
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
        sorted_contours = sorted(filtered_contours, key=cv2.contourArea, reverse=True)

        # If there are more than one contours exist
        if sorted_contours is not None:
            if len(sorted_contours) >= 2:
                # Get the largest and second largest contours
                max_contour = sorted_contours[0]
                second_max_contour = sorted_contours[1]

                # Get bounding rectangles
                x1, y1, w1, h1 = cv2.boundingRect(max_contour)
                x2, y2, w2, h2 = cv2.boundingRect(second_max_contour)

                # Merge the largest two bounding rectangles
                self.x = min(x1, x2)
                self.y = min(y1, y2)
                self.w = max(x1 + w1, x2 + w2) - self.x
                self.h = max(y1 + h1, y2 + h2) - self.y

                return image[self.y:self.y+self.h, self.x:self.x+self.w]

            # if there are only one contour
            elif len(sorted_contours) == 1:
                max_contour = sorted_contours[0]
                self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)
                return image[self.y:self.y+self.h, self.x:self.x+self.w]

        else:
            self.x, self.y, self.w, self.h = [0,0,image.shape[1],image.shape[0]]
            return image
        

    ##  Crop all images and extract features
    def extract_features(self, original_image: np.ndarray):
        # Enhence the edge first before color filter
        image = self.edg_enhence(original_image) 

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges for colors
        lb_G, ub_G = np.array([50, 60, 60]), np.array([130, 180, 200])  # Green
        lb_B, ub_B = np.array([90, 40, 30]), np.array([135, 150, 140])  # Blue
        lb_R = np.array([150, 120, 90])    # lower bound for Red
        ub_R = np.array([200, 250, 280])   # upper bound for Red
        
        # Create masks for each color
        mask_G = cv2.inRange(hsv, lb_G, ub_G)
        mask_B = cv2.inRange(hsv, lb_B, ub_B)
        mask_R = cv2.inRange(hsv, lb_R, ub_R)
        
        # Combine masks
        mask = cv2.bitwise_or(cv2.bitwise_or(mask_G, mask_B), mask_R)

        # Apply mask to the image
        image = cv2.bitwise_and(image, mask=mask)

        # Find contours and cropped image
        cropped_img = self.process_contours(image, mask)
        
        return cropped_img

    
    def run(self, original_img):
        # this handle_serviceline reads in all images listed in the file in color, and resizes them to 25x33 pixels
        train = np.array([np.array(cv2.resize( self.x_enhencement( self.extract_features(cv2.imread(self.imageDirectory+self.train_lines[i][0]+self.imageType)) ),(25,33) ) ) for i in range(len(self.train_lines))])
        # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants), note the *3 is due to 3 channels of color.

        train_data = train.flatten().reshape(len(self.train_lines), 33*25*3)
        train_data = train_data.astype(np.float32)

        # read in training labels
        train_labels = np.array([np.int32(self.train_lines[i][1]) for i in range(len(self.train_lines))])

        ## Train classifier
        self.knn = cv2.ml.KNearest_create()
        self.knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

        k = 3

        ## Read original img from camera and processing (cropping)
        processed_img = np.array(cv2.resize( self.x_enhencement( self.extract_features(original_img) ),(25,33) ))

        img_data = processed_img.flatten().reshape(1, 33*25*3)
        img_data = processed_img.astype(np.float32)

        ret, results, neighbours, dist = self.knn.findNearest(img_data, k)

        # Implement weighted voting
        weighted_votes = {}
        for idx, (neighbor, distance) in enumerate(zip(neighbours[0], dist[0])):
            weight = 1 / (distance ** 4 + 1e-5)  # Avoid division by zero with a small epsilon
            neighbor_class = int(neighbor)
            if neighbor_class in weighted_votes:
                weighted_votes[neighbor_class] += weight
            else:
                weighted_votes[neighbor_class] = weight
                 
        # Determine the class with the highest weighted vote
        ret = max(weighted_votes, key=weighted_votes.get)

        return ret, processed_img


class Camera_Recognition_Node(Node):
    def __init__(self, model: KNNClassifier):
        super().__init__('Camera_Recognition_Node')

        # Create a timer that triggers every 1 second (1.0 seconds)
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Create a service for main client to be called for
        self.srv = self.create_service(SetBool, 'camera_recognition_service', self.handle_service)

        # Subscribe the image for recognition
        image_qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=1)
        
        self.raw_image_subscriber=self.create_subscription(
            msg_type=Image,
            topic='/image_raw',
            callback=self.raw_image_callback,
            qos_profile=image_qos_profile
        )

        self.recog_img_publisher = self.create_publisher(
            msg_type=Image,
            topic='/recognition_img',
            qos_profile=image_qos_profile
        )

        self.pub_flag = False
        self.check_img = []
        self.recog_result = 0
        self.request = False
        self.recog_mode = model
        self.label_dic = {
                            "Empty": 0,
                            "left" : 1,
                            "right": 2,
                            "do not enter": 3,
                            "stop": 4,
                            "goal": 5,
                        }
       

    def timer_callback(self):
        if self.request:
            result = self.label_dic[self.recog_result]
            self.get_logger().info(f"Recognition Result = {self.recog_result}")
            self.pub_flag = True

        
    def handle_service(self, request, response):
        self.recog_result, self.check_img = self.recog_mode.image_recognition(self.recog_image)
        self.request = request.data

        response.success = True # Still now finished
        response.message = self.recog_result

        if self.pub_flag:
            ros_img = CvBridge().cv2_to_imgmsg(self.check_img,"bgr8")
            self.recog_img_publisher.publish(ros_img)
            self.pub_flag = False

        return response

    
    def raw_image_callback(self,ROS_frame:Image): # Load image
        self.recog_image = CvBridge().imgmsg_to_cv2(ROS_frame, "bgr8")
 
        

def main(args=None):
    rclpy.init(args=args)

    node = Camera_Recognition_Node(recog_mode)
    recog_mode = KNNClassifier()
    recog_mode.run(node.recog_image)

    node.get_logger().info("Model training completed!")
    rclpy.spin(node)