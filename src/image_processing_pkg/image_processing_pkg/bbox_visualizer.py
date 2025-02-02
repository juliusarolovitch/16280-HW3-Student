import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from object_detection_interfaces.msg import DetectionArray
import message_filters # For Sync!

class BBoxVisualizer(Node):
    def __init__(self):
        super().__init__('bbox_visualizer')
        
        self.bridge = CvBridge()
        
        # For synchronization btw images and predictions 
        self.image_sub = message_filters.Subscriber(self, Image, '/image_raw')
        self.detection_sub = message_filters.Subscriber(self, DetectionArray, '/detections')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.detection_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.publisher = self.create_publisher(Image, '/image_with_bboxes', 10)

        self.labels_dict = {1: "blue", 2: "green", 3: "red"}
        self.label_colors = {
            "blue": (255, 0, 0),
            "green": (0, 255, 0),
            "red": (0, 0, 255)
        }

        self.get_logger().info("BBox Visualizer Node Initialized")

    def callback(self, image_msg, detections_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        for detection in detections_msg.detections:
            x_min, y_min, x_max, y_max = map(int, detection.bbox)
            class_label = self.labels_dict.get(detection.label, "unknown")
            color = self.label_colors.get(class_label, (255, 255, 255))
            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(cv_image, f"{class_label}: {detection.score:.2f}", (x_min, y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(processed_msg)
        self.get_logger().info("Published processed image with bounding boxes")

def main(args=None):
    rclpy.init(args=args)
    node = BBoxVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()