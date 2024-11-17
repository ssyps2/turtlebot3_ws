#!/usr/bin/env python3
import cv2
import sys
import csv
import time
import math
import numpy as np
import random
import image_preproccess

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
        self.x, self.y, self.w, self.h, self.non_sign_num = [0,0,0,0,0]
        self.accuracy = 0.0
    

    ## Crop all images and extract features
    def extract_features(self, image):
        self.x, self.y,self.w, self.h, color_resized_image, non_sign_flag = image_preproccess.image_preprocess(image)
        self.non_sign_num += non_sign_flag
        return color_resized_image
        
    
    def run(self):
        # this line reads in all images listed in the file in color, and resizes them to 25x33 pixels
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
            print("ret = " + str(ret))

    
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
                print(str(self.lines[i][0]) + " Correct, " + str(ret))
                correct += 1
                confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
                print("\tweighted_vodes: " + str(weighted_votes))
              
            else:
                confusion_matrix[test_label][np.int32(ret)] += 1
                
                print(str(self.test_lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
                print("\tneighbours: " + str(neighbours))
                print("\tdistances: " + str(dist))
                wrong_path = np.append(wrong_path, self.imageDirectory+self.test_lines[i][0]+self.imageType)
                name = np.append(name, str(test_label)+ " classified as " + str(ret) )
                                

        print("\n\nTotal accuracy: " + str(correct/len(self.test_lines)))
        print(confusion_matrix)
        # print(self.non_sign_num)
        # for i in range(len(wrong_path)):
        #     cv2.imshow(name[i], cv2.imread(wrong_path[i]))
        
        # print(f"num of failures = {len(wrong_path)}")
        # cv2.waitKey(0)
        self.accuracy = correct/len(self.test_lines)

def main():
    
    run_num = 6
    accuracy_sum = 0.0
    for i in range(run_num):
        node = KNNClassifier()
        node.run()
        accuracy_sum += node.accuracy
    print("\n\n Ave accuracy: " + str(accuracy_sum/run_num))


main()