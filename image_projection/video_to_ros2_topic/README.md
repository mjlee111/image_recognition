# Video to ROS2 Topic Node

The `video_to_ros2_topic` package is a ROS 2 node that reads video files (such as MP4, AVI) from a specified path and publishes each frame to a ROS 2 topic at a configurable rate. This node allows users to easily publish video streams to ROS topics for testing and other purposes.

## Key Features

- **Video File Publishing**: The node reads frames from a specified video file and publishes them to a ROS topic.
- **ROS2 Integration**: Publishes frames as `sensor_msgs/msg/Image` messages that can be used in any ROS2 system.

## Prerequisites

Before using this package, make sure the following dependencies are installed:

1. ROS 2 Foxy or later
2. [cv_bridge](https://github.com/ros-perception/vision_opencv)
3. [OpenCV](https://opencv.org/) (for video processing)

### Installing Dependencies

To install the required libraries, run the following commands:

```bash
$ sudo apt update
$ sudo apt install ros-${ROS_DISTRO}-cv-bridge python3-opencv
```

## Installation

1. Clone the repository into your ROS 2 workspace:
   ```bash
   $ cd ~/colcon_ws/src
   $ git clone https://github.com/mjlee111/image_recognition.git
   ```

2. Build the workspace:
   ```bash
   $ cd ~/colcon_ws
   $ colcon build
   ```

3. Source the workspace:
   ```bash
   $ source ~/colcon_ws/install/setup.bash
   ```

## Launching the Node

To launch the `video_to_ros2_topic_node`, use the provided launch file. You can load parameters from a YAML file or pass them directly as launch arguments.

Example launch command:
```bash
$ ros2 launch video_to_ros2_topic video_to_topic.py
```

## Parameters

The following parameters can be configured when launching the node:

| Parameter Name      | Type    | Default Value                  | Description                                                      |
|---------------------|---------|--------------------------------|------------------------------------------------------------------|
| `video_topic`        | `string`| `/video_to_topic/image_raw`   | The ROS topic to publish the video frames to.                    |
| `video_path`         | `string`| `<package_share>/videos/test.mp4` | The file path to the video to be published.                      |

## ROS Topics

| Topic Name                      | Message Type                     | Role                                         |
|----------------------------------|-----------------------------------|----------------------------------------------|
| **/video_to_topic/image_raw**    | `sensor_msgs/msg/Image`           | Publishes the video frames from the specified file. |

## Example Usage

Launch the video publisher node with custom parameters:
```bash
$ ros2 launch video_to_ros2_topic video_to_topic.py video_path:=/path/to/your/video.mp4 video_topic:=/custom_video_topic
```

This command will publish the video located at `/path/to/your/video.mp4` to `/custom_video_topic`.
