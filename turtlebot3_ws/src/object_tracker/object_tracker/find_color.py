#!/usr/bin/env python3

import cv2
import numpy as np

frame = None

# Callback function to handle mouse events
def get_color(event, x, y, flags, param):
    global frame
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Read the pixel color in BGR
        bgr_color = frame[y, x]
        
        # Convert BGR to RGB
        rgb_color = tuple(bgr_color[::-1])
        
        # Convert BGR to HSV
        hsv_color = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        
        # Print the color values
        #print(f"Pixel location: ({x}, {y})")
        #print(f"RGB color: {rgb_color}")
        print(f"HSV color: {tuple(hsv_color)}")


def main():
    global frame
    
    # Start video capture (0 for default camera, or use a video file path)
    cap = cv2.VideoCapture(0)

    # Create a window and set the mouse callback function
    cv2.namedWindow('Frame')
    cv2.setMouseCallback('Frame', get_color)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break
        
        # Display the frame
        cv2.imshow('Frame', frame)
        
        # Exit if the user presses the ESC key
        if cv2.waitKey(1) & 0xFF == 27:
            break

    # Release the capture and destroy windows
    cap.release()
    cv2.destroyAllWindows()
