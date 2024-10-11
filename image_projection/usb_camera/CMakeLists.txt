# USB_CAMERA_NODE_CMAKELISTS.txt

cmake_minimum_required(VERSION 3.5)
project(usb_camera)

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

include_directories(include)

# Build usb_camera_node
add_executable(usb_camera_node src/usb_camera_node.cpp src/usb_camera.cpp)
ament_target_dependencies(usb_camera_node rclcpp sensor_msgs std_msgs cv_bridge rclcpp_lifecycle OpenCV)

# Build usb_camera_viewer_node
add_executable(usb_camera_viewer_node src/usb_camera_viewer.cpp)
ament_target_dependencies(usb_camera_viewer_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  usb_camera_node
  usb_camera_viewer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
