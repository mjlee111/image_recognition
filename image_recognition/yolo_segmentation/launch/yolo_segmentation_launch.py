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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('yolo_segmentation')

    model_path = os.path.join(package_share_directory, 'model', 'yolo11n-seg.pt')
    class_path = os.path.join(package_share_directory, 'model', 'class.txt')

    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/image_to_topic/image_raw')
    use_gpu_arg = DeclareLaunchArgument('use_gpu', default_value='False')
    image_encoding_arg = DeclareLaunchArgument('image_encoding', default_value='bgr8')
    yolo_version_arg = DeclareLaunchArgument('yolo_version', default_value='v8')

    return LaunchDescription([
        image_topic_arg,
        use_gpu_arg,
        image_encoding_arg,
        yolo_version_arg,
        Node(
            package='yolo_segmentation',
            executable='yolo_segmentation_node',
            name='yolo_segmentation_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'use_gpu': LaunchConfiguration('use_gpu'),
                'model_path': model_path,
                'class_path': class_path,
                'image_encoding': LaunchConfiguration('image_encoding'),
                'yolo_version': LaunchConfiguration('yolo_version'),
            }]
        )
    ])