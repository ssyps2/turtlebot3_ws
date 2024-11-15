#!/usr/bin/env python3
import cv2
import sys
import csv
import time
import math
import numpy as np
import random

class KNNClassifier:
    def __init__(self, imageDirectory='./2024F_imgs/', imageType='.png'):
        self.imageDirectory = imageDirectory
        self.imageType = imageType

        with open(imageDirectory + 'labels.txt', 'r') as f:
            self.reader = csv.reader(f)
            self.lines = list(self.reader)

        ## Randomly choose train and test data (50/50 split)
        random.shuffle(self.lines)
        self.train_lines = self.lines[:math.floor(len(self.lines)/2)][:]
        self.test_lines = self.lines[math.floor(len(self.lines)/2):][:]    # This will be camera captured image in demo

        # Cropped image coordinate
        self.x, self.y, self.w, self.h = [0,0,0,0]
    

    ## Crop all images and extract features
    def extract_features(self, image):
        # Convert BGR image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # In HSV space
        lb_G = np.array([50, 120, 100])    # lower bound for green
        ub_G = np.array([100, 180, 180])   # upper bound for green
        
        lb_B = np.array([90, 40, 40])      # lower bound for blue
        ub_B = np.array([150, 100, 130])   # upper bound for blue

        lb_R = np.array([150, 180, 90])    # lower bound for Red
        ub_R = np.array([200, 250, 280])   # upper bound for Red

        mask_G = cv2.inRange(hsv, lb_G, ub_G)
        mask_B = cv2.inRange(hsv, lb_B, ub_B)
        mask_R = cv2.inRange(hsv, lb_R, ub_R)

        mask = cv2.bitwise_or(cv2.bitwise_or(mask_G,mask_B),mask_R)

        # Find contours
        result = cv2.bitwise_and(image, image, mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding box around the contour with the largest area
        max_area = 0
        max_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)
            return image[self.y:self.y+self.h, self.x:self.x+self.w]
        else:
            self.x, self.y, self.w, self.h = [0,0,image.shape[1],image.shape[0]]
            return image
        
    
    def run(self):
        # this line reads in all images listed in the file in color, and resizes them to 25x33 pixels
        train = np.array([np.array(cv2.resize(self.extract_features(cv2.imread(self.imageDirectory+self.train_lines[i][0]+self.imageType)),(25,33))) for i in range(len(self.train_lines))])

        # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants), note the *3 is due to 3 channels of color.
        train_data = train.flatten().reshape(len(self.train_lines), 33*25*3)
        train_data = train_data.astype(np.float32)

        # read in training labels
        train_labels = np.array([np.int32(self.train_lines[i][1]) for i in range(len(self.train_lines))])

        ## Train classifier
        self.knn = cv2.ml.KNearest_create()
        self.knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

        if(__debug__):
            Title_images = 'Original Image'
            Title_resized = 'Image Resized'
            cv2.namedWindow( Title_images, cv2.WINDOW_AUTOSIZE )

        correct = 0.0
        confusion_matrix = np.zeros((6,6))

        k = 7

        for i in range(len(self.test_lines)):
            original_img = cv2.imread(self.imageDirectory+self.test_lines[i][0]+self.imageType)
            extracted_img = self.extract_features(original_img)
            #test_img = np.array(cv2.resize(self.extract_features(original_img),(25,33)))

            cv2.rectangle(original_img, (self.x, self.y), (self.x+self.w, self.y+self.h), (0, 255, 0), 2)
            print([self.x, self.y, self.w, self.h])

            if(__debug__):
                cv2.imshow(Title_images, original_img)
                cv2.imshow(Title_resized, extracted_img) #test_img
                
                key = cv2.waitKey()
                if key==27:    # Esc key to stop
                    break

        #     test_img = test_img.flatten().reshape(1, 33*25*3)
        #     test_img = test_img.astype(np.float32)

        #     test_label = np.int32(self.test_lines[i][1])

        #     ret, results, neighbours, dist = self.knn.findNearest(test_img, k)
        #     print("ret = " + str(ret))

        #     if test_label == ret:
        #         print(str(self.lines[i][0]) + " Correct, " + str(ret))
        #         correct += 1
        #         confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
        #     else:
        #         confusion_matrix[test_label][np.int32(ret)] += 1
                
        #         print(str(self.test_lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
        #         print("\tneighbours: " + str(neighbours))
        #         print("\tdistances: " + str(dist))

        # print("\n\nTotal accuracy: " + str(correct/len(self.test_lines)))
        # print(confusion_matrix)

def main():
    node = KNNClassifier()
    node.run()

main()