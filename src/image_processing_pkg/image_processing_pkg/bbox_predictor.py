import os
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_detection_interfaces.msg import DetectionArray, Detection


from torchvision.models.detection.faster_rcnn import fasterrcnn_resnet50_fpn

def load_model(checkpoint_path, device='cuda'):
    model = fasterrcnn_resnet50_fpn(pretrained=False, max_size=256, min_size=256, num_classes=4)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()
    return model

class BBoxPredictor(Node):
    def __init__(self):
        super().__init__('bbox_predictor')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.detection_publisher = self.create_publisher(DetectionArray, '/detections', 10)
        
        self.bridge = CvBridge()
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        model_path = "/home/turtle/16280-Object-Detection/src/image_processing_pkg/image_processing_pkg/model.pth"
        self.model = load_model(model_path, device=self.device)

        self.get_logger().info("BBox Predictor Node Initialized")

    def image_callback(self, msg):
        self.get_logger().info("Received an image for prediction")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1).float()
        image_tensor = image_tensor.to(self.device)
        images = [image_tensor]

        with torch.no_grad():
            predictions = self.model(images)

        detection_msg = DetectionArray()

        # Sync timestamp with image
        detection_msg.header.stamp = msg.header.stamp  

        for box, label, score in zip(predictions[0]['boxes'], predictions[0]['labels'], predictions[0]['scores']):
            if score < 0.9:
                continue
            detection = Detection()
            detection.bbox = [box[0].item(), box[1].item(), box[2].item(), box[3].item()]
            detection.label = int(label.item())
            detection.score = float(score.item())
            detection_msg.detections.append(detection)

        self.detection_publisher.publish(detection_msg)
        self.get_logger().info(f"Published {len(detection_msg.detections)} detections")

def main(args=None):
    rclpy.init(args=args)
    node = BBoxPredictor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()