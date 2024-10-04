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

#ifndef JOY_TO_PAN_TILT_NODE_HPP_
#define JOY_TO_PAN_TILT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "image_recognition_msgs/msg/pan_tilt_msgs.hpp"
#include "image_recognition_msgs/msg/pan_tilt_status_msgs.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/header.hpp"

#include <algorithm>

class Joy : public rclcpp::Node
{
public:
  Joy();

private:
  rclcpp::Publisher<image_recognition_msgs::msg::PanTiltMsgs>::SharedPtr pan_tilt_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<image_recognition_msgs::msg::PanTiltStatusMsgs>::SharedPtr
    status_subscriber_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void status_callback(const image_recognition_msgs::msg::PanTiltStatusMsgs::SharedPtr msg);

  bool is_init = false;
  int pan = -1;
  int tilt = -1;
  int steps[2] = {-1, -1};
  int pan_range[2] = {-1, -1};
  int tilt_range[2] = {-1, -1};
  std::string frame_id;
};

#endif  // PAN_TILT_CONTROL_HPP_
