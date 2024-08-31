#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    # Start video capture (or use a video file path)
    cap = cv2.VideoCapture(0)

    # Define the lower and upper bounds of the color in HSV space
    lower_bound = np.array([30, 150, 50])   # lower bound for green
    upper_bound = np.array([80, 200, 180])  # upper bound for green

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to read")
            break

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
            print(f"Centeral Coordinate: {(x+w*.5, y+h*.5)}")

        # Display the original frame and the result
        cv2.imshow('Frame', frame)
        #cv2.imshow('Mask', mask)
        #cv2.imshow('Result', result)

        # Exit if the user presses the ESC key
        if cv2.waitKey(1) & 0xFF == 27:
            break

    # Release the capture and destroy windows
    cap.release()
    cv2.destroyAllWindows()
