#!/usr/bin/env python3

import cv2
import numpy as np

image = cv2.imread('./2024F_imgs/298.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
x, y, w, h = [0,0,0,0]

def get_color(event, x, y, flags, param):
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Read the pixel color in BGR
        bgr_color = hsv[y, x]

        print(f"HSV color: {bgr_color}")


# In HSV space
lb_G = np.array([50, 120, 65])     # lower bound for green
ub_G = np.array([100, 180, 190])   # upper bound for green

lb_B = np.array([90, 40, 30])      # lower bound for blue
ub_B = np.array([135, 150, 140])   # upper bound for blue

lb_R = np.array([150, 120, 90])    # lower bound for Red
ub_R = np.array([200, 250, 280])   # upper bound for Red

mask_G = cv2.inRange(hsv, lb_G, ub_G)
mask_B = cv2.inRange(hsv, lb_B, ub_B)
mask_R = cv2.inRange(hsv, lb_R, ub_R)

mask = cv2.bitwise_or(cv2.bitwise_or(mask_G,mask_B),mask_R)
cv2.namedWindow('Mask')
cv2.imshow('Mask',mask)

# Find contours
result = cv2.bitwise_and(image, image, mask)

contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw bounding box around the contour with the largest area
max_area = 50
max_contour = None

for contour in contours:
    area = cv2.contourArea(contour)
    print(area)
    if area > max_area:
        max_area = area
        max_contour = contour

if max_contour is not None:
    x, y, w, h = cv2.boundingRect(max_contour)

cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 1)

cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', get_color)
cv2.imshow('Frame',image)
cv2.waitKey(0)
cv2.destroyAllWindows()