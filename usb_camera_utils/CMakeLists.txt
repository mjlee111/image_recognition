# USB_CAMERA_UTILS_CMAKELISTS_TXT

cmake_minimum_required(VERSION 3.5)
project(usb_camera_utils)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Build usb_camera_node
add_executable(usb_camera_utils_node src/usb_camera_utils.cpp)
ament_target_dependencies(usb_camera_utils_node rclcpp sensor_msgs std_msgs cv_bridge rclcpp_lifecycle)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  usb_camera_utils_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
