# DUAL_FISHEYE_CAMERA_CMAKELISTS_TXT

cmake_minimum_required(VERSION 3.5)
project(dual_fisheye_camera)

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

# Build fisheye_image_control_node
add_executable(fisheye_image_control_node src/image_ctrl.cpp)
ament_target_dependencies(fisheye_image_control_node rclcpp sensor_msgs std_msgs cv_bridge rclcpp_lifecycle)

# Build image_undistort_node
add_executable(image_undistort_node src/image_undistort.cpp)
ament_target_dependencies(image_undistort_node rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(image_undistort_node ${OpenCV_LIBS})

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
  fisheye_image_control_node
  image_undistort_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
