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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    left_image_topic = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/insta360air/left/camera/image_raw',
        description='Image topic to subscribe to'
    )
    
    right_image_topic = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/insta360air/right/camera/image_raw',
        description='Image topic to subscribe to'
    )
    camera_info_dir = os.path.join(
        get_package_share_directory('dual_fisheye_camera'),
        'config',
        'sample_insta360_air_left_info.yaml'  
    )

    
    launch_nodes = [
        left_image_topic,
        Node(
            package='dual_fisheye_camera',
            executable='image_undistort_node',
            name='image_undistort_node_left',  
            output='screen',
            parameters=[camera_info_dir, {'image_topic': LaunchConfiguration('left_image_topic')}]
        )
    ]
    launch_nodes.append(right_image_topic)
    launch_nodes.append(
        Node(
            package='dual_fisheye_camera',
            executable='image_undistort_node',
            name='image_undistort_node_right',  
            output='screen',
            parameters=[camera_info_dir, {'image_topic': LaunchConfiguration('right_image_topic')}]
        )
    )
    
    return LaunchDescription(launch_nodes)
