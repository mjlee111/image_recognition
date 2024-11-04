# USB Camera Node

This is a ROS2 Lifecycle node that interfaces with a USB camera using the V4L2 interface. The node provides streaming capabilities in various formats, resolutions, and frame rates, and allows dynamic reconfiguration of camera settings like brightness, contrast, white balance, and more. 

## Features

- Supports multiple video formats, including MJPEG, YUYV, H264, and more.
- Allows dynamic setting of camera controls (e.g., brightness, contrast, white balance).
- Streams both raw images and compressed images.
- Publishes camera data on customizable ROS topics.
- Handles lifecycle transitions to activate, deactivate, and clean up camera resources.

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

To run the `usb_camera_node`, use the provided launch file. It supports loading parameters from a YAML file or directly through launch arguments.

Example launch command:
```bash
$ ros2 launch usb_camera usb_camera_launch.py
```

Viewer launch command:
```bash
$ ros2 launch usb_camera image_viewer.py image_topic:=/your/ros/image/topic
```

![usb_camera_viewer_sample](../../docs/usb_camera/usb_camera_sample.jpg)

## Parameters

The node supports various parameters that can be configured via a YAML file or command line arguments. Here's a table of parameters:

| Parameter Name       | Type     | Default Value       | Description                                                                 |
|----------------------|----------|---------------------|-----------------------------------------------------------------------------|
| `topic`              | `string` | `/camera/image_raw`  | Topic to publish raw images.                                                |
| `compressed_topic`   | `string` | `/camera/compressed_image` | Topic to publish compressed images.                                         |
| `compressed_depth_topic` | `string` | `/camera/compressed_depth` | Topic to publish compressed depth images.                                    |
| `device`             | `string` | `/dev/video0`        | The path to the video device.                                               |
| `frame_id`           | `string` | `camera`            | Frame ID for the images.                                                    |
| `resolution`         | `string` | `1920x1080`         | Camera resolution in `<width>x<height>` format.                             |
| `fps`                | `float`  | `30.0`              | Frames per second.                                                          |
| `format`             | `string` | `MJPEG`             | Video format. Supported: MJPEG, YUYV, H264, etc.                            |
| `brightness`         | `int`    | `-1`                | Brightness setting of the camera (range depends on camera).                 |
| `contrast`           | `int`    | `-1`                | Contrast setting of the camera.                                             |
| `saturation`         | `int`    | `-1`                | Saturation setting of the camera.                                           |
| `hue`                | `int`    | `-1`                | Hue setting of the camera.                                                  |
| `gamma`              | `int`    | `-1`                | Gamma setting of the camera.                                                |
| `sharpness`          | `int`    | `-1`                | Sharpness setting of the camera.                                            |
| `whitebalance`       | `int`    | `-1`                | White balance setting (manual mode).                                        |
| `auto_whitebalance`  | `bool`   | `true`              | Whether to enable automatic white balance.                                  |
| `exposure`           | `int`    | `-1`                | Exposure setting (manual mode).                                             |
| `auto_exposure`      | `bool`   | `true`              | Whether to enable automatic exposure.                                       |
| `focus`              | `int`    | `-1`                | Focus setting (manual mode).                                                |
| `auto_focus`         | `bool`   | `true`              | Whether to enable automatic focus.                                          |
| `zoom`               | `int`    | `-1`                | Zoom level of the camera.                                                   |
| `pan`                | `int`    | `-1`                | Pan setting of the camera.                                                  |
| `tilt`               | `int`    | `-1`                | Tilt setting of the camera.                                                 |
| `rotate`             | `int`    | `-1`                | Rotation angle of the image.                                                |
| `horizontal_flip`    | `bool`   | `false`             | Whether to horizontally flip the image.                                     |
| `vertical_flip`      | `bool`   | `false`             | Whether to vertically flip the image.                                       |

## Example YAML Configuration

```yaml
/**:
  ros__parameters:
    topic: "/camera/image_raw"
    compressed_topic: "/camera/compressed_image"
    compressed_depth_topic: "/camera/compressed_depth"
    device: "/dev/video0"
    frame_id: "camera"
    resolution: "1920x1080"
    fps: 30.0
    format: "MJPEG"
    brightness: -1
    contrast: -1
    saturation: -1
    hue: -1
    gamma: -1
    sharpness: -1
    whitebalance: -1
    auto_whitebalance: true
    exposure: -1
    auto_exposure: true
    focus: -1
    auto_focus: true
    zoom: -1
    pan: -1
    tilt: -1
    rotate: -1
    horizontal_flip: false
    vertical_flip: false
```
You can easily set parameters from using `v4l2-ctl`
```shell
$ sudo apt install v4l2-ctl
$ v4l2-ctl --device=/dev/video0 --all # change /dev/video0 to your device
```
