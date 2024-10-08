# Image Recognition
Stack for image recognition packages based on ROS2.

<div align="center">
  
[![Build and Test - Foxy](https://github.com/mjlee111/image_recognition/actions/workflows/foxy.yml/badge.svg?branch=master&event=push)](https://github.com/mjlee111/image_recognition/actions/workflows/foxy.yml)[![Build and Test - Humble](https://github.com/mjlee111/image_recognition/actions/workflows/humble.yml/badge.svg?branch=master&event=push)](https://github.com/mjlee111/image_recognition/actions/workflows/humble.yml)

</div>

## Overview
This repository provides a set of packages designed for image recognition and camera control using ROS2. Each package is designed with a focus on modularity, lifecycle management, and real-time control over image streams.

## Requirements
To use the packages in this repository, make sure you have the following installed:

- **OpenCV**: Version **4.2** or higher  
  Required for image processing and camera interfacing.

- **ROS2**:  
  ROS2 distribution, such as **Foxy** or **Humble**, is recommended.

## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill     |
| **OpenCV**  | 4.10.0            |


## Packages
<div align="center">

| Category          | Package              | Description                                                         | Documentation                                        |
|-------------------|----------------------|---------------------------------------------------------------------|-----------------------------------------------------|
| Camera Nodes      | `usb_camera`          | V4L2 USB Camera Node with lifecycle management.                     | [Link to docs](usb_camera/README.md)                |
| Image Projection  | `pan_tilt_camera`     | Advanced V4L2 USB Camera Node with lifecycle management and pan-tilt control via ROS topics. | [Link to docs](image_projection/pan_tilt_camera/README.md) |
| Image Projection  | `image_to_ros2_topic` | Publishes static image files (JPG, PNG etc) to ROS2 topics with configurable parameters.          | [Link to docs](image_projection/image_to_ros2_topic/README.md)        |
| Image Projection  | `video_to_ros2_topic` | Publishes static video files (MP4, AVI etc) to ROS2 topics with configurable parameters.          | [Link to docs](image_projection/video_to_ros2_topic/README.md)        |
| Image Recognition  | `yolov8_detection`    | Real-time image recognition using YOLOv8, with GPU support and dynamic parameter configuration. | [Link to docs](image_recognition/yolov8_detection/README.md)          |


</div>

## Contributing
I welcome all contributions! Whether it's bug reports, feature suggestions, or pull requests, your input helps me to improve. If you're interested in contributing, please check out my contributing guidelines or submit an issue.

## License
This project is licensed under the [Apache 2.0 License](LICENSE). Feel free to use and distribute it according to the terms of the license.

## Contact
If you have any questions or feedback, don't hesitate to reach out! You can contact us at [menggu1234@naver.com][email].

[email]: mailto:menggu1234@naver.com
