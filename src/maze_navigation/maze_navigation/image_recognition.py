#!/usr/bin/env python3
import cv2
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge #add dependency


class Image_Recognition(Node):
    def __init__(self):
        super().__init__('image_recognition')

        self.imageDirectory = '/home/pengyuan/Desktop/turtlebot3_ws/src/maze_navigation/maze_navigation/2024F_imgs/'
        self.imageType = '.png'

        with open(self.imageDirectory + 'labels.txt', 'r') as f:
            self.reader = csv.reader(f)
            self.lines = list(self.reader)

        self.train_lines = self.lines[:math.floor(len(self.lines))][:]

        self.x, self.y, self.w, self.h= [0,0,0,0]

        self.train_flag = 0
        self.recog_result = 0

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.raw_image_subscriber=self.create_subscription(CompressedImage,'/image_raw/compressed',self.raw_image_callback,qos_profile)

        self.cropped_img_pub = self.create_publisher(Image,'/cropped_img',10)
        self.recog_result_pub = self.create_publisher(Int32,'/recog_label',10)
        self.img_center_pub = self.create_publisher(Float64,'/img_center_angle',10)


    ## Edge enhencement to original image
    def edg_enhence(self, image:np.ndarray) -> np.ndarray:
        x, y, channel = np.shape(image) 
        enhenced_image = np.zeros([x,y,channel],dtype=np.uint8)

        for i in range(channel):
            channel_image = image[:,:,i]
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
    def x_enhencement(self, image:np.ndarray):
        # Apply Sobel filter to detect x-direction gradients
        sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)  # Gradient in x-direction
        sobel_x = np.absolute(sobel_x) * 3  # Take the absolute value to remove negative gradients
        sobel_x = cv2.convertScaleAbs(sobel_x)   # Convert back to uint8


        # Normalize the result to enhance differences
        sobel_x_normalized = cv2.normalize(sobel_x, None, 0, 255, cv2.NORM_MINMAX)
        combined_image = cv2.addWeighted(image, 0.7, sobel_x_normalized, 0.3, 0)
        # combined_image = cv2.GaussianBlur(combined_image, (5, 5), 1.4)
        return combined_image


    ## Contours processing
    def process_contours(self, image, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area in descending order
        min_contour_area = 50
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
        sorted_contours = sorted(filtered_contours, key=cv2.contourArea, reverse=True)

        center_ang_msg = Float64()

        self.get_logger().info(f"length of sorted_contours: {len(sorted_contours)}")

        if (sorted_contours is not None) & (self.recog_result != 0):
            # If there are more than one contours exist
            if len(sorted_contours) >= 2:
                # Get the largest and second largest contours
                max_contour = sorted_contours[0]
                # second_max_contour = sorted_contours[1]

                # # Get bounding rectangles
                # x1, y1, w1, h1 = cv2.boundingRect(max_contour)
                # x2, y2, w2, h2 = cv2.boundingRect(second_max_contour)

                # # Merge the largest two bounding rectangles
                # self.x = min(x1, x2)
                # self.y = min(y1, y2)
                # self.w = max(x1 + w1, x2 + w2) - self.x
                # self.h = max(y1 + h1, y2 + h2) - self.y

                self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)

            # if there are only one contour
            elif len(sorted_contours) == 1:
                max_contour = sorted_contours[0]
                self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)

            # max_contour = sorted_contours[0]
            # self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)

        else:
            self.x, self.y, self.w, self.h = [0,0,image.shape[1],image.shape[0]]

        self.get_logger().info(f"x,y: {self.x},{self.y}")
        
        center_ang_msg.data = -(62.2*(self.x+self.w*0.5-image.shape[1]*0.5)/(image.shape[1]*0.5)) # in deg
        self.img_center_pub.publish(center_ang_msg)

        return image[self.y:self.y+self.h, self.x:self.x+self.w]
        

    ##  Crop all images and extract features
    def extract_features(self, image):
        if self.original_img is None:
            raise ValueError("Input image is None. Check the file path.")
    
        # Enhence the edge first before color filter
        #image = self.edg_enhence(image) 

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
        result_img = cv2.bitwise_and(image, image, mask=mask)

        if len(result_img.shape) < 2 or result_img.shape[0] == 0 or result_img.shape[1] == 0:
            raise ValueError("result_img has invalid dimensions.")

        # Find contours and cropped image
        cropped_img = self.process_contours(result_img, mask)
        
        # Publish cropped image
        if len(cropped_img.shape) < 2 or cropped_img.shape[0] == 0 or cropped_img.shape[1] == 0:
            raise ValueError("Cropped image has invalid dimensions. Ensure it is not empty.")
        else:
            ros_image = CvBridge().cv2_to_imgmsg(cropped_img,"bgr8")
            self.cropped_img_pub.publish(ros_image)

        return cropped_img

    
    def run(self):
        if self.train_flag == 0:
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

            self.train_flag = 1

        k = 5

        ## Read original img from camera and processing (cropping)
        processed_img = np.array(cv2.resize( self.x_enhencement(self.extract_features(self.original_img)),(25,33) ))

        img_data = processed_img.flatten().reshape(1, 33*25*3)
        img_data = img_data.astype(np.float32)

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
        self.recog_result = ret

        self.get_logger().info(f"State: {self.recog_result}")

        # Publish the recognition result
        result_msg = Int32()
        result_msg.data = ret
        self.recog_result_pub.publish(result_msg)


    def raw_image_callback(self,msg:CompressedImage):
        try:
            self.original_img = CvBridge().compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

        ## Crop the original image smaller
        # self.crop_f = 0.5 # crop_factor
        # w,h = [img.shape[1],img.shape[0]]
        # self.original_img = img[round(h*self.crop_f*0.5):round(h*(1-self.crop_f*0.5)),
        #                     round(w*self.crop_f*0.5):round(w*(1-self.crop_f*0.5))]

        self.run()

        

def main(args=None):
    rclpy.init(args=args)
    image_recognition_node=Image_Recognition()
    image_recognition_node.get_logger().info("Model Created")

    while rclpy.ok():
        rclpy.spin_once(image_recognition_node)
    
    image_recognition_node.destroy_node()
    rclpy.shutdown()