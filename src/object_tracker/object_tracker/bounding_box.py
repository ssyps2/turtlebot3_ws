#!/usr/bin/env python3

import cv2

# Initialize global variables
drawing = False
box = None
start_point = None
tracker = None
frame = None

# Mouse callback function to draw rectangle
def draw_rectangle(event, x, y, flags, param):
    global start_point, box, drawing, tracker, frame

    if event == cv2.EVENT_LBUTTONDOWN:
        # When left mouse button is pressed, start drawing a rectangle
        drawing = True
        start_point = (x, y)
        box = None
        tracker = None  # Reset the tracker
        print("Tracker Reset")

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            # Update the rectangle as the mouse moves
            end_point = (x, y)
            box = (start_point[0], start_point[1], x - start_point[0], y - start_point[1])

    elif event == cv2.EVENT_LBUTTONUP:
        # Finalize the rectangle when the left mouse button is released
        drawing = False
        end_point = (x, y)
        box = (start_point[0], start_point[1], x - start_point[0], y - start_point[1])

        # Initialize the tracker
        tracker = cv2.legacy.TrackerMOSSE_create()
        tracker.init(frame, box)


def main():
    global start_point, box, drawing, tracker, frame
    
    # Initialize video capture
    cap = cv2.VideoCapture(0)

    # Capture the first frame
    ret, frame = cap.read()

    # Create a window and bind the mouse callback function to it
    cv2.namedWindow('Tracking')
    cv2.setMouseCallback('Tracking', draw_rectangle)
    
    while True:
        # Capture a new frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to read")
            break

        # Draw the rectangle if the user is dragging
        if box is not None:
            if drawing:  # While drawing, show the rectangle being drawn
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            elif tracker is not None:  # Update the tracker and box if not drawing
                success, box = tracker.update(frame)
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    print(f"Centeral Coordinate: {(x+w*.5, y+h*.5)}")

        # Display the frame
        cv2.imshow('Tracking', frame)

        # Exit if the user presses the ESC key
        if cv2.waitKey(1) & 0xFF == 27:
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
