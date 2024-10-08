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
import time

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        self.declare_parameter('image_topic', 'image_topic')
        self.declare_parameter('video_path', '/path/to/your/video.mp4')

        self.image_topic = self.get_parameter('image_topic').value
        self.video_path = self.get_parameter('video_path').value

        self.publisher_ = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()

        if not os.path.exists(self.video_path):
            self.get_logger().error(f"Video file does not exist: {self.video_path}")
            rclpy.shutdown()

        self.capture = cv2.VideoCapture(self.video_path)
        if not self.capture.isOpened():
            self.get_logger().error('Failed to open video file')
            rclpy.shutdown()

        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.total_frames = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.duration = self.total_frames / self.fps

        self.get_logger().info(f'Found video file: {self.video_path}')
        self.get_logger().info(f'Video size: {self.width}x{self.height}')
        self.get_logger().info(f'Video fps: {self.fps}')
        self.get_logger().info(f'Total video duration: {self.duration:.2f} seconds')

    def publish_video(self):
        frame_time = 1.0 / self.fps  

        while rclpy.ok():
            ret, frame = self.capture.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(ros_image)
                self.get_logger().info(f'Frame published to {self.image_topic}')

                time.sleep(frame_time)
            else:
                self.get_logger().info('End of video stream')
                break

        self.capture.release()  
        rclpy.shutdown()  

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()

    node.publish_video() 

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
