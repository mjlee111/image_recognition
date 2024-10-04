# PAN_TILT_CAMERA_CMAKELISTS_TXT

cmake_minimum_required(VERSION 3.5)
project(pan_tilt_camera)

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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_recognition_msgs REQUIRED)

include_directories(include)

# Build pan_tilt_camera_node
add_executable(pan_tilt_camera_node src/pan_tilt_camera_node.cpp src/usb_camera.cpp)
ament_target_dependencies(pan_tilt_camera_node rclcpp sensor_msgs std_msgs cv_bridge image_recognition_msgs rclcpp_lifecycle)

# Build joy control node
add_executable(joy_to_pan_tilt_node src/joy_to_pan_tilt_node.cpp)
ament_target_dependencies(joy_to_pan_tilt_node rclcpp std_msgs sensor_msgs image_recognition_msgs)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  pan_tilt_camera_node
  joy_to_pan_tilt_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
