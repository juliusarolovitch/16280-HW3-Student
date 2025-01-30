import os
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from torchvision.models.detection.faster_rcnn import fasterrcnn_resnet50_fpn

def load_model(checkpoint_path, device='cuda'):
    model = fasterrcnn_resnet50_fpn(pretrained=False, max_size=256, min_size=256, num_classes=4)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()
    return model

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10  
        )
        self.publisher = self.create_publisher(Image, '/image_with_bboxes', 10)
        self.bridge = CvBridge()
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        model_path = "/home/turtle/cv_ws/src/image_processing_pkg/image_processing_pkg/model.pth"
        self.model = load_model(model_path, device=self.device)

        self.labels = {1: "blue", 2: "green", 3: "red"}
        self.label_colors = {
            "blue": (255, 0, 0),
            "green": (0, 255, 0),
            "red": (0, 0, 255)
        }
        
        self.get_logger().info("Image Processor Node Initialized")

    def image_callback(self, msg):
        self.get_logger().info("Received an image")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).float()
        image_tensor = image_tensor.to(self.device)
        images = [image_tensor]

        with torch.no_grad():
            predictions = self.model(images)

        for box, label, score in zip(predictions[0]['boxes'], predictions[0]['labels'], predictions[0]['scores']):
            if score < 0.9:  
                continue
            x_min, y_min, x_max, y_max = box.int().cpu().numpy()
            class_label = self.labels.get(label.item(), "unknown")
            color = self.label_colors.get(class_label, (255, 255, 255))
            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(cv_image, f"{class_label}: {score:.2f}", (x_min, y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.25, color, 1)

        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(processed_msg)
        self.get_logger().info("Published processed image with bounding boxes")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
