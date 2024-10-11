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

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_directory = get_package_share_directory('yolo_detection')

    model_path = os.path.join(package_share_directory, 'model', 'yolov11n.pt')
    class_path = os.path.join(package_share_directory, 'model', 'class.txt')

    return LaunchDescription([
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            output='screen',
            parameters=[{
                'image_topic': '/image_to_topic/image_raw',
                'use_gpu': False,
                'model_path': model_path,
                'class_path': class_path,
                'image_encoding': 'bgr8',
                'yolo_version': 'v8',
            }]
        )
    ])
