import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
from torch.utils.data import Dataset, DataLoader
import cv2
import os
from PIL import Image

# Define YOLO Model
class YOLO(nn.Module):
    def __init__(self, num_classes=6, num_anchors=3):
        super(YOLO, self).__init__()
        self.num_classes = num_classes
        self.num_anchors = num_anchors

        # Backbone (Feature Extractor)
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(16),
            nn.LeakyReLU(0.1),
            nn.MaxPool2d(kernel_size=2, stride=2),

            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(32),
            nn.LeakyReLU(0.1),
            nn.MaxPool2d(kernel_size=2, stride=2),

            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.LeakyReLU(0.1),
            nn.MaxPool2d(kernel_size=2, stride=2),

            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(128),
            nn.LeakyReLU(0.1),
            nn.MaxPool2d(kernel_size=2, stride=2),

            nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(256),
            nn.LeakyReLU(0.1),
            nn.MaxPool2d(kernel_size=2, stride=2),
        )

        # Detection Head
        self.head = nn.Sequential(
            nn.Conv2d(256, 512, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(512),
            nn.LeakyReLU(0.1),
            nn.Conv2d(512, num_anchors * (5 + num_classes), kernel_size=1, stride=1, padding=0),
        )

    def forward(self, x):
        features = self.backbone(x)
        output = self.head(features)
        
        # Get dimensions dynamically
        batch_size, _, height, width = output.shape
        grid_size = height  # Assuming height == width (common in YOLO)

        # Validate the total number of elements
        num_elements = batch_size * grid_size * grid_size * self.num_anchors * (5 + self.num_classes)
        assert output.numel() == num_elements, (
            f"Mismatch in output size. Expected {num_elements}, but got {output.numel()}. "
            f"Shape of output: {output.shape}"
        )

        # Reshape
        output = output.permute(0, 2, 3, 1).contiguous()
        output = output.view(batch_size, grid_size, grid_size, self.num_anchors, 5 + self.num_classes)
        return output


# Custom Dataset
class CustomDataset(Dataset):
    def __init__(self, image_paths, labels, transform=None, target_size=(224,224)):
        self.image_paths = image_paths
        self.labels = labels
        self.transform = transform
        self.target_size = target_size

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        # Read image
        image = cv2.imread(self.image_paths[idx])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, self.target_size)

        # Convert to tensor
        if self.transform:
            image = self.transform(image)
        else:
            image = torch.tensor(image, dtype=torch.float32).permute(2, 0, 1) / 255.0

        label = self.labels[idx]
        return image, torch.tensor(label, dtype=torch.float32)


# YOLO Loss
class YOLOLoss(nn.Module):
    def __init__(self):
        super(YOLOLoss, self).__init__()

    def forward(self, predictions, targets):
        # Placeholder: Implement actual YOLO loss
        loss = torch.mean((predictions - targets) ** 2)
        return loss


def compute_classification_accuracy(pred_classes, true_classes):
    correct = sum([1 for p, t in zip(pred_classes, true_classes) if p == t])
    return correct / len(true_classes)

# Training Function
def model_train():
    # Hyperparameters
    num_classes = 6
    anchors = [[1.0, 2.0], [2.0, 1.0], [1.5, 1.5]]
    epochs = 10
    batch_size = 1
    learning_rate = 0.001

    # Paths and Labels
    ImageDirectory = '/home/pengyuan/Desktop/turtlebot3_ws/src/maze_navigation/maze_navigation/2024F_imgs/'
    imageType = '.png'

    with open(os.path.join(ImageDirectory, 'labels.txt'), 'r') as file:
        lines = file.readlines()

    image_paths = []
    labels = []
    for line in lines:
        values = line.strip().split(",")
        image_id, label = values[0], int(values[1])
        image_paths.append(os.path.join(ImageDirectory, image_id + imageType))
        labels.append([label])  # Example format, adjust as needed

    # Dataset and DataLoader
    dataset = CustomDataset(image_paths, labels, transform=transforms.ToTensor())
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    # Model, Loss, Optimizer
    model = YOLO(num_classes=num_classes, num_anchors=len(anchors))
    loss_fn = YOLOLoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    # Training Loop
    for epoch in range(epochs):
        model.train()
        total_loss = 0
        for images, targets in dataloader:
            optimizer.zero_grad()

            predictions = model(images)
            loss = loss_fn(predictions, targets)

            loss.backward()
            optimizer.step()

            total_loss += loss.item()
           
        print(f"Epoch {epoch + 1}/{epochs}, Loss: {total_loss / len(dataloader)}")
    torch.save(model.state_dict(), "yolo_model.pth")


def post_process(output, confidence_threshold=0.5, iou_threshold=0.5):
    # Filter predictions based on confidence
    predictions = []
    for pred in output[0]:  # Assuming output shape is [batch_size, grid_size, grid_size, anchors, 5 + num_classes]
        for box in pred:
            conf = box[4]  # confidence score
            if conf > confidence_threshold:
                # x, y, w, h = box[:4]  # bounding box coordinates
                class_scores = box[5:]  # class probabilities
                class_id = torch.argmax(class_scores)  # class with max probability
                # predictions.append((x, y, w, h, conf, class_id.item()))
    
    return predictions


def evaluate():
     # Load the trained model
    model = YOLO(num_classes=6, num_anchors=3)
    model.load_state_dict(torch.load("yolo_model.pth"))
    model.eval()  # Set to evaluation mode

    # Preprocess the image
    image_path = "/home/pengyuan/Desktop/Lab6_2/2024F_imgs/130.png"
    image = Image.open(image_path)

    # Define the transformations: resize, convert to tensor, and normalize
    transform = transforms.Compose([
        transforms.Resize((224, 224)),  # Resize to the input size the model expects
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # If your model was trained with this normalization
    ])

    image = transform(image).unsqueeze(0)  # Add batch dimension

    # Make the prediction
    with torch.no_grad():
        outputs = model(image)

    # Post-process the output (get bounding boxes and class)
    predictions = post_process(outputs)

    # Print predictions
    for prediction in predictions:
        print(f"Class: {prediction[5]}, Confidence: {prediction[4]:.2f}, Box: {prediction[:4]}")



if __name__ == "__main__":
    # model_train()
    evaluate()
