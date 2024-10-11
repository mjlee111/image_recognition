/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef USB_CAMERA_NODE_HPP
#define USB_CAMERA_NODE_HPP

#include "usb_camera.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

class UsbCameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  UsbCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UsbCameraNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  void get_param();

  void set_param(std::string parameter_name, int parameter_value);
  void set_param_auto(
    std::string parameter_name, std::string auto_parameter_name, int parameter_value,
    bool auto_parameter_value);
  void set_camera();

  void publish_camera_info(const std_msgs::msg::Header & header);
  std::shared_ptr<usb_cam> camera_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr
    compressed_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CompressedImage>::SharedPtr
    compressed_depth_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_change_callback_;

  // Camera Config Params
  std::string topic, camera_name, compressed_topic, compressed_depth_topic, camera_info_topic,
    camera_path, frame_id, resolution_str, format;
  double fps;
  int brightness, contrast, saturation, hue, gamma, sharpness, whitebalance, exposure, focus, zoom,
    pan, tilt, rotate;
  int image_width, image_height;
  bool auto_whitebalance, auto_exposure, auto_focus, horizontal_flip, vertical_flip;

  // Camera Info Params
  std::string distortion_model;
  std::vector<double> camera_matrix, distortion_coeffs, rectification_matrix, projection_matrix;
};

#endif  // USB_CAMERA_NODE_HPP
