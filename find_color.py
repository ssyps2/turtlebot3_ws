#!/usr/bin/env python3

import cv2
import numpy as np

imageDirectory = './2024F_imgs/237.png'
frame = cv2.imread(imageDirectory)
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

def get_color(event, x, y, flags, param):
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Read the pixel color in BGR
        bgr_color = hsv_frame[y, x]

        print(f"HSV color: {bgr_color}")

cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', get_color)
cv2.imshow('Frame',frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
