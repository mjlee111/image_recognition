# image_recognition_msgs

This package contains custom message definitions for `image_recognition` package and camera control in ROS 2.

## Message Types
### CameraInfoMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |

### VideoDeviceCtrlMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |
| focus | int32 | Focus setting |
| auto_focus | bool | Auto focus flag |
| zoom | int32 | Zoom setting |
| pan_tilt_msgs | PanTiltMsgs | Embedded PanTiltMsgs |
| horizontal_flip | bool | Horizontal flip flag |
| vertical_flip | bool | Vertical flip flag |

### ImageCtrlMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |
| brightness | int32 | Brightness setting |
| contrast | int32 | Contrast setting |
| saturation | int32 | Saturation setting |
| hue | int32 | Hue setting |
| gamma | int32 | Gamma setting |
| sharpness | int32 | Sharpness setting |
| whitebalance | int32 | White balance setting |
| auto_whitebalance | bool | Auto white balance flag |
| exposure | int32 | Exposure setting |
| auto_exposure | bool | Auto exposure flag |

### PanTiltMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |
| pan | int32 | Pan value |
| tilt | int32 | Tilt value |

### PanTiltStatusMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |
| pan | int32 | Current pan value |
| tilt | int32 | Current tilt value |
| pan_range | int32[2] | Min and max pan values [min, max] |
| tilt_range | int32[2] | Min and max tilt values [min, max] |
| steps | int32[2] | Step values for pan and tilt [pan_step, tilt_step] |

### CoordinateMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| xy | int32[2] | Array of 2 values [x, y] |

### BoundingBoxMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |
| class_id | int32 | ID of the detected class |
| box | float64[4] | Array of 4 values [xmin, ymin, xmax, ymax] |

### SegmentationMsgs.msg

| Name | Data Type | Description |
|------|-----------|-------------|
| header | std_msgs/Header | Standard ROS 2 message header |
| class_id | int32 | ID of the segmented class |
| mask | uint8[,] | Mask of the segmented class |


## Usage

To use these messages in your ROS 2 package, add `image_recognition_msgs` as a dependency in your `package.xml` file:

```xml
<depend>image_recognition_msgs</depend>
```

Then, in your C++ or Python code, you can include and use these messages as needed.

For C++:
```cpp
#include "image_recognition_msgs/msg/bounding_box_msgs.hpp"
```

For Python:
```python
from image_recognition_msgs.msg import BoundingBoxMsgs
```

Replace `BoundingBoxMsgs` with the specific message type you need to use.
