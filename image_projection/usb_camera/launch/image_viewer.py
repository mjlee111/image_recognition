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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera1/camera/image_raw',
            description='The topic for the image stream to subscribe to'
        ),
        DeclareLaunchArgument(
            'viewer_name',
            default_value='usb_camera_viewer',
            description='Name of the viewer node'
        ),

        # Launch the viewer node
        Node(
            package='usb_camera',
            executable='usb_camera_viewer_node',
            name=LaunchConfiguration('viewer_name'),
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
            }]
        )
    ])
