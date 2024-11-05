# /*
#  * Copyright 2024 Myeong Jin Lee
#  *
#  * Licensed under the Apache License, Version 2.0 (the "License");
#  * you may not use this file except in compliance with the License.
#  * You may obtain a copy of the License at
#  *
#  *     http://www.apache.org/licenses/LICENSE-2.0
#  *
#  * Unless required by applicable law or agreed to in writing, software
#  * distributed under the License is distributed on an "AS IS" BASIS,
#  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  * See the License for the specific language governing permissions and
#  * limitations under the License.
#  */

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import os
import cv2
import random
from image_recognition_msgs.msg import BoundingBoxMsgs, CoordinateMsgs
import sys

os.environ['YOLO_VERBOSE'] = 'False'

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('model_path', 'yolov11n.pt')
        self.declare_parameter('class_path', 'class.txt')
        self.declare_parameter('image_encoding', 'bgr8')
        self.declare_parameter('yolo_version', 'v11')


        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        class_path = self.get_parameter('class_path').get_parameter_value().string_value
        yolo_version = self.get_parameter('yolo_version').get_parameter_value().string_value

        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            rclpy.shutdown()
            sys.exit(1)
        else:
            self.get_logger().info(f'Model file found: {model_path}')

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10
        )
        
        self.get_logger().info(f'Subscribing to topic : {image_topic}')

        output_box_topic = f"{image_topic}/yolo_bboxes"
        output_image_topic = f"{image_topic}/yolo_output"
        self.bounding_box_publisher = self.create_publisher(BoundingBoxMsgs, output_box_topic, 10)
        self.image_publisher = self.create_publisher(Image, output_image_topic, 10)

        self.bridge = CvBridge()
        self.model = self.initialize_yolo_model(model_path, yolo_version)
        self.get_logger().info(f'YOLO model initialized: {self.model}')
        self.classes = None

        if class_path and os.path.exists(class_path):
            with open(class_path, 'r') as f:
                self.classes = f.read().strip().split('\n')
                if len(self.classes) == 0:
                    self.get_logger().warn(
                        f'Class file {class_path} is empty, using class indices.'
                    )
                else:
                    self.get_logger().info(f'Classes loaded from {class_path}')
        else:
            self.get_logger().warn(f'Class file not found: {class_path}, using class indices.')

        if use_gpu:
            if torch.cuda.is_available():
                self.get_logger().info('Using GPU for YOLO inference.')
                self.device = 'cuda'
            else:
                self.get_logger().warn('No GPU available. Falling back to CPU.')
                self.device = 'cpu'
        else:
            self.get_logger().info('Using CPU for YOLO inference.')
            self.device = 'cpu'

        self.model.to(self.device)

        self.class_colors = {}
        self.generate_class_colors()
        
    def initialize_yolo_model(self, model_path, yolo_version):
        if yolo_version.lower() == 'v8':
            self.get_logger().warn(f'Unsupported YOLO version: {yolo_version}. Using YOLOv8 as default.')
            return YOLO(model_path, verbose=False)
        else:
            return YOLO(model_path, verbose=False)

    def generate_class_colors(self):
        """Generate random colors for each class."""
        if self.classes:
            for i, class_name in enumerate(self.classes):
                self.class_colors[class_name] = (
                    random.randint(0, 255), 
                    random.randint(0, 255), 
                    random.randint(0, 255)
                )
        else:
            for i in range(254):  
                self.class_colors[str(i)] = (
                    random.randint(0, 255), 
                    random.randint(0, 255), 
                    random.randint(0, 255)
                )

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                self.get_parameter('image_encoding').get_parameter_value().string_value
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        try:
            results = self.model.predict(source=cv_image, device=self.device, show=False)
        except Exception as e:
            self.get_logger().error(f'YOLO prediction failed: {e}')
            return

        if len(results) == 0 or len(results[0].boxes) == 0:
            self.get_logger().info('No objects detected.')
            return

        for i, (cls, bbox) in enumerate(zip(results[0].boxes.cls, results[0].boxes.xyxy)):
            bounding_box_msg = BoundingBoxMsgs()
            bounding_box_msg.header.frame_id = msg.header.frame_id
            bounding_box_msg.header.stamp = self.get_clock().now().to_msg()
            bounding_box_msg.box = [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])]

            class_idx = int(cls)
            class_name = self.classes[class_idx] if self.classes and class_idx < len(self.classes) else str(class_idx)
            bounding_box_msg.class_id = class_idx

            self.bounding_box_publisher.publish(bounding_box_msg)
            self.get_logger().info(f'Published bounding box: {bounding_box_msg}')

            color = self.class_colors.get(class_name, (0, 255, 0))  

            cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), color, 2)
            label = f"{class_name}"
            cv2.putText(cv_image, label, (int(bbox[0]), int(bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


        try:
            img_msg_with_boxes = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_publisher.publish(img_msg_with_boxes)
            self.get_logger().info('Published image with bounding boxes.')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image with bounding boxes: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
