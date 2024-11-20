import cv2
import numpy as np


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

    image = edg_enhence(original_image) # Enhence the edge first before color filter


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

    x, y , w, h, color_resized_image, non_sign_flag = color_resize(original_image)

    edg_enhenced_image = edg_enhence(color_resized_image)
    gray_image = cv2.cvtColor(edg_enhenced_image, cv2.COLOR_BGR2GRAY)
    # descriptors = SIFT_process(gray_image)

    return x, y, w, h, color_resized_image, non_sign_flag


# origin_image = cv2.imread('./2024F_imgs/19.png')
# gray_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2GRAY)
# blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0.5)
# sobel_y = cv2.Sobel(blurred_image, cv2.CV_64F, 0, 1, ksize=3,scale=2)  # Gradient in y
# blurred_y = cv2.GaussianBlur(sobel_y, (5, 5), 0.5)
# cv2.imshow("edge enhenced",blurred_y)
# cv2.imshow("Original Image", origin_image)
# cv2.waitKey(0)

# origin_image = cv2.imread('./2024F_imgs/185.png')
# x, y, w, h, filtered_image, non_sign_flag = color_resize(origin_image)
# detect_signs(filtered_image)

def main():
    {}