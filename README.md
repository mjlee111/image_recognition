# Image Recognition
Stack for image recognition packages based on ROS2.

<div align="center">
<img src="https://img.shields.io/badge/ROS-22314E?style=flat&logo=cplusplus&logoColor=white"/> 
</div>

## Overview
This repository provides a set of packages designed for image recognition and camera control using ROS2. Each package is designed with a focus on modularity, lifecycle management, and real-time control over image streams.

## Packages
<div align="center">

| Category          | Package           | Description                                                         | Documentation                                        | Build Status |
|-------------------|-------------------|---------------------------------------------------------------------|-----------------------------------------------------|--------------|
| Camera Nodes      | `usb_camera`      | V4L2 USB Camera Node with lifecycle management.                     | [Link to docs](usb_camera/README.md)                | ![usb_camera build](https://github.com/mjlee111/image_recognition/actions/workflows/ci.yml/badge.svg?branch=master&event=push&label=usb_camera) |
| Image Projection  | `pan_tilt_camera` | Advanced V4L2 USB Camera Node with lifecycle management and pan-tilt control via ROS topics. | [Link to docs](image_projection/pan_tilt_camera/README.md) | ![pan_tilt_camera build](https://github.com/mjlee111/image_recognition/actions/workflows/ci.yml/badge.svg?branch=master&event=push&label=pan_tilt_camera) |


</div>

## Contributing
I welcome all contributions! Whether it's bug reports, feature suggestions, or pull requests, your input helps us improve. If you're interested in contributing, please check out our contributing guidelines or submit an issue.

## License
This project is licensed under the [Apache 2.0 License](LICENSE). Feel free to use and distribute it according to the terms of the license.

## Contact
If you have any questions or feedback, don't hesitate to reach out! You can contact us at [menggu1234@naver.com][email].

[email]: mailto:menggu1234@naver.com
