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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'status_topic',
            default_value='/camera1/pan_tilt_status',
            description='Topic to subscribe pan and tilt status'
        ),
        DeclareLaunchArgument(
            'control_topic',
            default_value='/camera1/pan_tilt',
            description='Topic to publish pan and tilt values'
        ),
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Topic to subscribe joy value'
        ),
        
        Node(
            package='pan_tilt_camera',
            executable='joy_to_pan_tilt_node',
            name='pan_tilt_control',
            output='screen',
            parameters=[],
            remappings=[
                ('/camera1/pan_tilt_status', LaunchConfiguration('status_topic')),
                ('/camera1/pan_tilt', LaunchConfiguration('control_topic')),
                ('/joy', LaunchConfiguration('joy_topic'))
            ]
        )
    ])
