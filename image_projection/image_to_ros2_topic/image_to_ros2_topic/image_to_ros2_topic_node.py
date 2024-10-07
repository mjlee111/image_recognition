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
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('image_topic', 'image_topic')
        self.declare_parameter('image_path', '/path/to/your/image.jpg')

        self.publish_period = self.get_parameter('publish_period').value
        self.image_topic = self.get_parameter('image_topic').value
        self.image_path = self.get_parameter('image_path').value

        self.publisher_ = self.create_publisher(Image, self.image_topic, 10)
        self.timer = self.create_timer(self.publish_period, self.timer_callback)
        self.bridge = CvBridge()

        if not os.path.exists(self.image_path):
            self.get_logger().error(f"Image file does not exist: {self.image_path}")
            rclpy.shutdown()

    def timer_callback(self):
        cv_image = cv2.imread(self.image_path)
        if cv_image is not None:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info(f'Image published to {self.image_topic}')
        else:
            self.get_logger().error('Failed to load image')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
