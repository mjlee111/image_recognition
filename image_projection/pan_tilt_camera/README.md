# Pan-Tilt Camera Node

The `pan_tilt_camera_node` is an enhanced ROS2 Lifecycle node built upon the `usb_camera_node`[documenation](../../usb_camera/README.md). It interfaces with a USB camera via the V4L2 API, providing pan and tilt control in addition to all the features of the original USB camera node. The node supports streaming in multiple formats, dynamic reconfiguration of camera settings, and advanced control over camera movements, making it ideal for more sophisticated applications such as surveillance, robotics, and interactive systems. Supports v4l2 pan tilt camera as `insta360 link`, `OBSBot` etc.

## Key Improvements Over `usb_camera_node`

- **Pan-Tilt Control**: Adds support for dynamically controlling pan and tilt axes of the camera using V4L2 control interfaces.
- **Enhanced Camera Control**: In addition to basic camera settings like brightness and contrast, the node offers precise control over camera orientation (pan, tilt) to dynamically adjust the field of view.
- **Lifecycle Management**: The node adheres to the ROS2 lifecycle management system, allowing safe activation, deactivation, and cleanup of camera resources.
- **Expanded ROS2 Topic Support**: Publishes camera images, control status, and metadata over various ROS2 topics, customizable through parameters.

## Features

- **Full Pan-Tilt Capabilities**: Use V4L2 control commands to dynamically adjust the camera's pan and tilt axes during operation.
- **Multiple Video Formats**: Supports MJPEG, YUYV, H264, and more.
- **Dynamic Reconfiguration**: Allows on-the-fly changes to camera settings such as brightness, contrast, white balance, and more via the parameter server or YAML files.
- **Compressed Image Support**: Publishes both raw and compressed images on configurable ROS topics.
- **ROS2 Lifecycle Support**: Ensures safe transitions between states and proper resource management, preventing issues during startup or shutdown.

## Installation

1. Clone the repository into your ROS2 workspace:
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

To launch the `pan_tilt_camera_node`, use the provided launch file. You can load parameters from a YAML file or pass them directly as launch arguments.

Example launch command:
```bash
$ ros2 launch pan_tilt_camera pan_tilt_camera.py
```

For viewing the image stream with a pan-tilt camera:
```bash
$ ros2 launch usb_camera image_viewer.py image_topic:=/your/ros/image/topic
```

## Using `v4l2-ctl` for Manual Configuration
To manually configure pan and tilt values or test other camera settings, you can use the `v4l2-ctl` command-line tool:

```bash
$ sudo apt install v4l2-ctl
$ v4l2-ctl --device=/dev/video0 --set-ctrl=pan_absolute=10000  # Example: Set pan to 10000
$ v4l2-ctl --device=/dev/video0 --set-ctrl=tilt_absolute=10000  # Example: Set tilt to 10000
```

## Sample Pan Tilt Control node with Joystick
Tested with `SONY Dual Shock 4` & `insta360 Link`

Launch command:
```bash
$ ros2 launch pan_tilt_camera joy_pan_tilt_control.py
```

![joy_sample](../../docs/image_projection/pan_tilt_joy_sample.gif)

