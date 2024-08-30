#!/usr/bin/env python3

import cv2

def main():
    
    openCvVidCapIds = []

    for i in range(10):
        try:
            cap = cv2.VideoCapture(i)
            if cap is not None and cap.isOpened():
                openCvVidCapIds.append(i)
        except:
            pass

    print(str(openCvVidCapIds))