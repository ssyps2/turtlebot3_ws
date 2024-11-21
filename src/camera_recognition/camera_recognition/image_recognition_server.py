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
from ament_index_python.packages import get_package_share_directory


class image_preproccess():

    def edg_enhence(image:np.ndarray) -> np.ndarray:
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


    def process_contours(image, original_image):
        # Find contours
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Ensure at least two contours exist
        if len(contours) >= 2:
            # Sort contours by area in descending order
            sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

            # Get the largest and second largest contours
            max_contour = sorted_contours[0]
            second_max_contour = sorted_contours[1]

            # Get bounding rectangles
            x1, y1, w1, h1 = cv2.boundingRect(max_contour)
            x2, y2, w2, h2 = cv2.boundingRect(second_max_contour)

            # Merge the two bounding rectangles
            x = min(x1, x2)
            y = min(y1, y2)
            w = max(x1 + w1, x2 + w2) - x
            h = max(y1 + h1, y2 + h2) - y

            # Draw the merged rectangle on the image
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv2.imshow("Merged Region", image)

            # Return the merged region
            return x, y, w, h, original_image[y:y+h, x:x+w], 0

        # If fewer than 2 contours exist, return a black image
        else:
            cv2.imshow("No Significant Region", image)
            return 0, 0, 0, 0, original_image * 0, 1


    def color_resize(original_image: np.ndarray):

        image = image_preproccess.edg_enhence(original_image) # Enhence the edge first before color filter


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

        # Crop the region of interest if a valid contour exists
        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)

            # if the region is very small, check if the region locates at relatively center of image
            area = float(w*h)
            if area < 120:
                x_ratio = (x + w/2) / image.shape[1]
                y_ratio = (y + h/2) / image.shape[0]
                if( (x_ratio > 0.7) or (x_ratio < 0.3) ) or ( (y_ratio > 0.7) or (y_ratio < 0.3) ):
                    x, y, w, h = 0, 0, image.shape[1], image.shape[0]  # Default to the entire image
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
                    print("Hello")
                    return x, y, w, h, original_image*0, 1  # Return black image if no significant region is found

            # elif area / float((image.shape[0]*image.shape[1])) > 0.7: # Too close
            #     cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
            #     cv2.imshow("edge enhenced image",image)
            #     return x, y, w, h, image[y:y+h, x:x+w], 0

            else:
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
                cv2.imshow("edge enhenced image",image)
                return x, y, w, h, image[y:y+h, x:x+w], 0
        # Cropped region, return original_image instead of enhenced image in order to avoid double effects of edge enhencement
    
        else:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
            cv2.imshow("edge enhenced image",image)
            return x, y, w, h, original_image*0, 1  # Return black image if no significant region is found


    def color_resize_recog(original_image: np.ndarray):

        image = image_preproccess.edg_enhence(original_image) # Enhence the edge first before color filter


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
        max_area = 1500
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
        
        x, y, w, h = 0, 0, image.shape[1], image.shape[0]  # Default to the entire image

        # Crop the region of interest if a valid contour exists
        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)

            # if the region is very small, check if the region locates at relatively center of image
            area = float(w*h)
    
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
            cv2.imshow("edge enhenced image",image)
            return x, y, w, h, original_image[y:y+h, x:x+w], 0
    
        else:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)
            cv2.imshow("edge enhenced image",image)
            return x, y, w, h, original_image*0, 1  # Return black image if no significant region is found





    def x_enhencement(origin_image:np.ndarray): # Enhence x gradient to tell the left and right arrow
        # Apply Sobel filter to detect x-direction gradients
        sobel_x = cv2.Sobel(origin_image, cv2.CV_64F, 1, 0, ksize=3)  # Gradient in x-direction
        sobel_x = np.absolute(sobel_x) * 3  # Take the absolute value to remove negative gradients
        sobel_x = cv2.convertScaleAbs(sobel_x)   # Convert back to uint8


        # Normalize the result to enhance differences
        sobel_x_normalized = cv2.normalize(sobel_x, None, 0, 255, cv2.NORM_MINMAX)
        combined_image = cv2.addWeighted(origin_image, 0.7, sobel_x_normalized, 0.3, 0)
        # combined_image = cv2.GaussianBlur(combined_image, (5, 5), 1.4)
        return combined_image


    def image_preprocess(original_image: np.ndarray ) -> np.ndarray:
        
    # Step 1: Load the color image

        x, y , w, h, color_resized_image, non_sign_flag = image_preproccess.color_resize(original_image)

        edg_enhenced_image = image_preproccess.edg_enhence(color_resized_image)
        gray_image = cv2.cvtColor(edg_enhenced_image, cv2.COLOR_BGR2GRAY)
        # descriptors = SIFT_process(gray_image)

        return x, y, w, h, color_resized_image, non_sign_flag


    


class KNNClassifier:
    def __init__(self, imageDirectory='./2024F_imgs/', imageType='.png'):
        
        
        script_dir = os.path.dirname(os.path.abspath(__file__))  # Path of the current script file

        # Define the relative path to the images folder
        image_directory = os.path.join(script_dir, '2024F_imgs')


        # Get the path to your package
        package_share_directory = get_package_share_directory('camera_recognition')

        # Construct the full path to your file
        file_path = os.path.join(package_share_directory, '/2024F_imgs/')

        imageDirectory = '/home/donatello/Desktop/Lab6_Cao/src/camera_recognition/camera_recognition/2024F_imgs/'

        self.imageDirectory = imageDirectory
        self.imageType = imageType

        with open(imageDirectory + 'labels.txt', 'r') as f:
            self.reader = csv.reader(f)
            self.lines = list(self.reader)

        ## Randomly choose train and test data (50/50 split)
        random.shuffle(self.lines)
        self.train_lines = self.lines[:math.floor(len(self.lines)/8)][:]
        self.test_lines = self.lines[math.floor(len(self.lines)/2):][:]    # This will be camera captured image in demo

        # Cropped image coordinate
        self.x, self.y, self.w, self.h, self.non_sign_num = [0,0,0,0,0]
        self.accuracy = 0.0

        self.recog_image = []

        self.label_dic = {
                            "Empty": 0,
                            "left" : 1,
                            "right": 2,
                            "do not enter": 3,
                            "stop": 4,
                            "goal": 5,
                        }
        self.reverse_label_dict = {v: k for k, v in self.label_dic.items()} # Inverted dictionary (one-time operation)
    

    ## Crop all images and extract features
    def extract_features(self, image):
        self.x, self.y,self.w, self.h, color_resized_image, non_sign_flag = image_preproccess.image_preprocess(image)
        self.non_sign_num += non_sign_flag
        return color_resized_image
        
    
    def run(self):
        # this handle_serviceline reads in all images listed in the file in color, and resizes them to 25x33 pixels
        train = np.array([np.array(cv2.resize( image_preproccess.x_enhencement( self.extract_features(cv2.imread(self.imageDirectory+self.train_lines[i][0]+self.imageType)) ),(25,33) ) ) for i in range(len(self.train_lines))])
        # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants), note the *3 is due to 3 channels of color.

        train_data = train.flatten().reshape(len(self.train_lines), 33*25*3)
        train_data = train_data.astype(np.float32)

        # read in training labels
        train_labels = np.array([np.int32(self.train_lines[i][1]) for i in range(len(self.train_lines))])

        ## Train classifier
        self.knn = cv2.ml.KNearest_create()
        self.knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

        # if(__debug__):
        #     Title_images = 'Original Image'
        #     Title_resized = 'Image Resized'
        #     cv2.namedWindow( Title_images, cv2.WINDOW_AUTOSIZE )

        correct = 0.0
        confusion_matrix = np.zeros((6,6))

        k = 7

        wrong_path = []
        name = []

        for i in range(len(self.test_lines)):
            original_img = cv2.imread(self.imageDirectory+self.test_lines[i][0]+self.imageType)
            #extracted_img = self.extract_features(original_img)
            test_img = np.array(cv2.resize(image_preproccess.x_enhencement(self.extract_features(original_img)),(25,33)))

            cv2.rectangle(original_img, (self.x, self.y), (self.x+self.w, self.y+self.h), (0, 255, 0), 1)

            # if(__debug__):
            #     cv2.imshow(Title_images, original_img)
            #     cv2.imshow(Title_resized, test_img)
            #     key = cv2.waitKey()
            #     if key==27:    # Esc key to stop
            #         break

            test_img = test_img.flatten().reshape(1, 33*25*3)
            test_img = test_img.astype(np.float32)

            test_label = np.int32(self.test_lines[i][1])

            ret, results, neighbours, dist = self.knn.findNearest(test_img, k)
            # print("ret = " + str(ret))

    
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

            if test_label == ret:
                # print(str(self.lines[i][0]) + " Correct, " + str(ret))
                correct += 1
                confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
                # print("\tweighted_vodes: " + str(weighted_votes))
              
            else:
                confusion_matrix[test_label][np.int32(ret)] += 1
                
                # print(str(self.test_lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
                # print("\tneighbours: " + str(neighbours))
                # print("\tdistances: " + str(dist))
                wrong_path = np.append(wrong_path, self.imageDirectory+self.test_lines[i][0]+self.imageType)
                name = np.append(name, str(test_label)+ " classified as " + str(ret) )
                                

        # print("\n\nTotal accuracy: " + str(correct/len(self.test_lines)))
        # print(confusion_matrix)
        # print(self.non_sign_num)
        # for i in range(len(wrong_path)):
        #     cv2.imshow(name[i], cv2.imread(wrong_path[i]))
        
        # print(f"num of failures = {len(wrong_path)}")
        # cv2.waitKey(0)
        self.accuracy = correct/len(self.test_lines)

    def image_recognition(self, recog_image: np.ndarray):

            check_img = recog_image
            original_img = recog_image

            x, y, w, h, cropped_image, _ = image_preproccess.color_resize_recog(original_img)
            test_img = np.array(cv2.resize(image_preproccess.x_enhencement(cropped_image),(25,33)))

            cv2.rectangle(original_img, (x, y), (x+w, y+h), (0, 255, 0), 1)

           

            test_img = test_img.flatten().reshape(1, 33*25*3)
            test_img = test_img.astype(np.float32)

            ret, results, neighbours, dist = self.knn.findNearest(test_img, k=7)

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

            result = self.reverse_label_dict.get(ret)


            

            return result, check_img # pass back the class name
        





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
        self.reverse_label_dict = {v: k for k, v in self.label_dic.items()} # Inverted dictionary (one-time operation)
       

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
 
   
    # Train the model first and then run node, since spin(node) will block following nodes
    recog_mode = KNNClassifier()
    recog_mode.run()

    
    rclpy.init(args=args)
    node = Camera_Recognition_Node(recog_mode)
    node.get_logger().info("Model training completed!")
    rclpy.spin(node)


if __name__ == '__main__':
    main()
