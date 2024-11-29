import torch
import torchvision.transforms as transforms
from PIL import Image
import cv2
from torchvision.models import resnet18
import numpy as np
import torch.nn as nn
import time
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge #add dependency


class MyResNet18(nn.Module):
    def __init__(self):
        """Initialize network layers.

        Note: Do not forget to freeze the layers of ResNet except the last one
        Note: Use 'mean' reduction in the loss_criterion. Read Pytorch documention to understand what it means

        Download pretrained resnet using pytorch's API (Hint: see the import statements)
        """
        super().__init__()

        self.conv_layers = None
        self.fc_layers = None
        self.loss_criterion = None

        ############################################################################
        # Student code begin
        ############################################################################

         # Load a pretrained ResNet-18 model
        model = resnet18(pretrained=True)

        # Freeze all layers except the last fully connected layer
        for param in model.parameters():
            param.requires_grad = False
            

        # Separate convolutional and fully connected layers
        self.conv_layers = nn.Sequential(*list(model.children())[:-1])  # All layers except the last FC
        
        for param in model.fc.parameters():
            param.requires_grad = True
        self.fc_layers = nn.Linear(model.fc.in_features, 15)  # Replace last FC layer (adjust output as needed)

        # Define the loss criterion with 'mean' reduction
        self.loss_criterion = nn.CrossEntropyLoss(reduction='mean')

        # raise NotImplementedError(
        #     "`__init__` function in "
        #     + "`my_resnet.py` needs to be implemented"
        # )

        ############################################################################
        # Student code end
        ############################################################################

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Perform the forward pass with the net, duplicating grayscale channel to 3-channel.

        Args:
            x: tensor of shape (N,C,H,W) representing input batch of images
        Returns:
            y: tensor of shape (N,num_classes) representing the output (raw scores) of the net
                Note: we set num_classes=15
        """
        model_output = None
        x = x.repeat(1, 3, 1, 1)  # as ResNet accepts 3-channel color images [N,1,H,W]-->[N,3,H,W]
        ############################################################################
        # Student code begin
        ############################################################################
        
        # Pass through the convolutional layers
        x = self.conv_layers(x)

        # Flatten the output for the fully connected layer
        x = torch.flatten(x, 1)  # Flatten starting from the second dimension, therefore, each instances in batch has their own flatterned output

        # Pass through the fully connected layers
        model_output = self.fc_layers(x)

        return model_output
        raise NotImplementedError(
            "`forward` function in "
            + "`my_resnet.py` needs to be implemented"
        )

        ############################################################################
        # Student code end
        ############################################################################
       

class Model_Functions():
    def color_croop(self, image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges for colors
        lb_G, ub_G = np.array([40, 60, 60]), np.array([130, 220, 230])  # Green
        lb_B, ub_B = np.array([90, 70, 30]), np.array([160, 150, 140])  # Blue
        lb_R = np.array([150, 120, 150])   # lower bound for Red
        ub_R = np.array([200, 250, 280])   # upper bound for Red
        
        # Create masks for each color
        mask_G = cv2.inRange(hsv, lb_G, ub_G)
        mask_B = cv2.inRange(hsv, lb_B, ub_B)
        mask_R = cv2.inRange(hsv, lb_R, ub_R)
        
        # Combine masks
        mask = cv2.bitwise_or(cv2.bitwise_or(mask_G, mask_B), mask_R)

        # Apply mask to the image
        result_img = cv2.bitwise_and(image, image, mask=mask)

        # Find contours and cropped image
        cropped_img = self.process_contours(result_img, mask)
        

        return cropped_img


    def process_contours(self, image, mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))  # Adjust the kernel size as needed
        mask = cv2.dilate(mask, kernel, iterations=3)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area in descending order
        min_contour_area = 50
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]
        sorted_contours = sorted(filtered_contours, key=cv2.contourArea, reverse=True)

        if (len(sorted_contours) != 0) :
            max_contour = sorted_contours[0]
            return image[y:y+h, x:x+w]
        else:
            x, y, w, h = [0,0,image.shape[1],image.shape[0]]
            return image*0


        # Gray scale
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)




    def load_model(self, model_path):
        
        model = MyResNet18()
        checkpoint = torch.load(model_path)  # Load the model
        state_dict = checkpoint["state_dict"]
        model.load_state_dict(state_dict)
        model.eval()  # Set the model to evaluation mode
        return model
    

    def standardize(self, array):
    
        mean = np.mean(array)
        std = np.std(array)
        standardized_array = (array - mean) / std
        return standardized_array


    def preprocess_image(self, image, input_size):

        transform = transforms.Compose([
            transforms.Resize((input_size, input_size)),  # Resize the image
            transforms.ToTensor(),  # Convert image to PyTorch tensor
            transforms.Normalize(mean=0.341383777191757,std=0.17576761555998005),  # Normalize
        ])

        # Color croop
        img_croop = self.color_croop(image)

        # Convert BGR to RGB
        img_rgb = cv2.cvtColor(img_croop, cv2.COLOR_BGR2RGB)

        # Convert NumPy array back to Pillow image
        img_pillow = Image.fromarray(img_rgb)

        # Convert the image to grayscale ('L' mode)
        img_return = img_pillow.convert("L")

        return transform(img_return).unsqueeze(0)  # Add batch dimension

        

    def model_evaluate(self, model,test_images, input_size):
        """
        Test the model with test images.

        Args:
            model: Loaded PyTorch model.
            test_images (list): List of test image paths.
            labels (dict): Dictionary of {filename: label}.
            input_size (int): The expected input size of the model.

        Returns:
            predicted value
        """
      
        input_tensor = self.preprocess_image(test_images, input_size)
        
        # Make predictions     

        with torch.no_grad():
            outputs = model(input_tensor)
            _, predicted = torch.max(outputs, 1)  # Get the predicted 
        
        return predicted.item()

      
       
     

class ResNet18Node(Node):
    def __init__(self):
        super().__init__('image_recognition')

        # Paths
        model_path = "./trained_MyResNet18_final.pt"  # Replace with your .pt file path
        self.input_size = 224  # Replace with the input size expected by your model

        # Load the model and labels
        self.model = Model_Functions.load_model(model_path)

        self.recog_result = 0

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.raw_image_subscriber=self.create_subscription(CompressedImage,'/image_raw/compressed',self.raw_image_callback,qos_profile)

        self.original_img_pub = self.create_publisher(Image,'/original_img',10)
        self.chopped_img_pub = self.create_publisher(Image,'/chopped_img',10)
        self.recog_result_pub = self.create_publisher(Int32,'/recog_label',10)
        self.img_center_pub = self.create_publisher(Float64,'/img_center_angle',10)



    def run(self):
        
        self.recog_result = Model_Functions.model_evaluate(input_size=self.input_size, model=self.model, test_images=self.original_img)
        
        # Publish the recognition result
        result_msg = Int32()
        result_msg.data = self.recog_result
        self.recog_result_pub.publish(result_msg)
        self.get_logger().info(f"Results = {self.recog_result}")
  

    def raw_image_callback(self,msg:CompressedImage):
        try:
            self.original_img = CvBridge().compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            
        self.run()

        

def main(args=None):
    rclpy.init(args=args)
    image_recognition_node=ResNet18Node()

    while rclpy.ok():
        rclpy.spin_once(image_recognition_node)
    
    image_recognition_node.destroy_node()
    rclpy.shutdown()