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

#ifndef USB_CAMERA_VIEWER_HPP
#define USB_CAMERA_VIEWER_HPP

#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

class UsbCameraViewer : public rclcpp::Node
{
public:
  UsbCameraViewer();
  ~UsbCameraViewer() = default;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  std::string image_topic;
};

#endif  // USB_CAMERA_VIEWER_HPP
