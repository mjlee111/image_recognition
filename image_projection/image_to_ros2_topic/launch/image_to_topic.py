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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_period',
            default_value='0.1',
            description='Publishing period in seconds'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_to_topic/image_raw',
            description='Image topic name'
        ),
        DeclareLaunchArgument(
            'image_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('image_to_ros2_topic'),
                'images',
                'test.jpg'
            ]),
            description='Path to the image file'
        ),

        Node(
            package='image_to_ros2_topic', 
            executable='image_to_ros2_topic_node',  
            name='image_to_topic_node',
            output='screen',
            parameters=[{
                'publish_period': LaunchConfiguration('publish_period'),
                'image_topic': LaunchConfiguration('image_topic'),
                'image_path': LaunchConfiguration('image_path'),
            }]
        )
    ])
