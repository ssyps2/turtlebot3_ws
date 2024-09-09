#!/usr/bin/env python3

import cv2

openCvVidCapIds = []

def main():

    for i in range(10):
        try:
            cap = cv2.VideoCapture(i)
            if cap is not None and cap.isOpened():
                openCvVidCapIds.append(i)
        except:
            pass

    print(str(openCvVidCapIds))