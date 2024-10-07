# Image to ROS2 Topic Node

The `image_to_ros2_topic` package is a ROS 2 node that reads image files (such as JPG, PNG) from a specified path and publishes them to a ROS 2 topic at a configurable rate. This node allows users to easily publish static images to ROS topics for testing and other purposes.

## Key Features

- **Image File Publishing**: The node reads images from a specified file path and publishes them to a ROS topic.
- **Configurable Publishing Rate**: The rate at which the images are published can be easily set via ROS parameters.
- **ROS2 Integration**: Publishes images as `sensor_msgs/msg/Image` messages that can be used in any ROS2 system.

## Prerequisites

Before using this package, make sure the following dependencies are installed:

1. ROS 2 Foxy or later
2. [cv_bridge](https://github.com/ros-perception/vision_opencv)
3. [OpenCV](https://opencv.org/) (for image processing)

### Installing Dependencies

To install the required libraries, run the following commands:

```bash
$ sudo apt update
$ sudo apt install ros-foxy-cv-bridge python3-opencv
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

To launch the `image_to_ros2_topic_node`, use the provided launch file. You can load parameters from a YAML file or pass them directly as launch arguments.

Example launch command:
```bash
$ ros2 launch image_to_ros2_topic image_to_topic.py
```

## Parameters

The following parameters can be configured when launching the node:

| Parameter Name      | Type    | Default Value                  | Description                                                      |
|---------------------|---------|--------------------------------|------------------------------------------------------------------|
| `publish_period`     | `float` | `0.1`                          | The rate at which the image is published, in seconds.             |
| `image_topic`        | `string`| `/image_to_topic/image_raw`     | The ROS topic to publish the image to.                           |
| `image_path`         | `string`| `<package_share>/images/test.jpg` | The file path to the image to be published.                      |

## ROS Topics

| Topic Name                      | Message Type                     | Role                                         |
|----------------------------------|-----------------------------------|----------------------------------------------|
| **/image_to_topic/image_raw**    | `sensor_msgs/msg/Image`           | Publishes the image data from the specified file. |

## Example Usage

Launch the image publisher node with custom parameters:
```bash
$ ros2 launch image_to_ros2_topic image_to_topic.py image_path:=/path/to/your/image.jpg publish_period:=0.5 image_topic:=/custom_image_topic
```

This command will publish the image located at `/path/to/your/image.jpg` to `/custom_image_topic` at 0.5-second intervals.
