# IMAGE_RECOGNITION_MSGS_CMAKELISTS_TXT

cmake_minimum_required(VERSION 3.5)
project(image_recognition_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/PanTiltMsgs.msg"
  "msg/PanTiltStatusMsgs.msg"
  "msg/CameraInfoMsgs.msg"
  "msg/VideoDeviceCtrlMsgs.msg"
  "msg/ImageCtrlMsgs.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs sensor_msgs geometry_msgs)

ament_package()
