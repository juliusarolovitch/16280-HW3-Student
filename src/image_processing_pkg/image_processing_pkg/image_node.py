import os
import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_detection_interfaces.msg import DetectionArray, Detection

SERVER_URL = "http://34.204.191.29:5000/predict" 

class ImageNode(Node):
    def __init__(self):
        super().__init__('image_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.detection_publisher = self.create_publisher(DetectionArray, '/detections', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("BBox Predictor Node Initialized (Using Remote Inference)")

    def image_callback(self, msg):
        self.get_logger().info("Received an image for prediction")

        # Convert ROS2 image message to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Encode image as JPEG to send to the server
        _, img_encoded = cv2.imencode(".jpg", cv_image)

        # Send the image to  server
        try:
            response = requests.post(SERVER_URL, files={"image": img_encoded.tobytes()})
            response.raise_for_status() 
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send request to server: {e}")
            return

        # Parse JSON response
        detections = response.json().get("detections", [])
        detection_msg = DetectionArray()
        detection_msg.header.stamp = msg.header.stamp

        for det in detections:
            detection = Detection()
            detection.bbox = det["bbox"]
            detection.label = det["label"]
            detection.score = det["score"]
            detection_msg.detections.append(detection)

        # Publish  detections
        self.detection_publisher.publish(detection_msg)
        self.get_logger().info(f"Published {len(detection_msg.detections)} detections")

def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
