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

#include "../include/pan_tilt_camera/joy_to_pan_tilt_node.hpp"

Joy::Joy() : Node("pan_tilt_control")
{
  std::string status_topic =
    this->declare_parameter<std::string>("status_topic", "/camera1/pan_tilt_status");
  std::string control_topic =
    this->declare_parameter<std::string>("control_topic", "/camera1/pan_tilt");
  std::string joy_topic = this->declare_parameter<std::string>("joy_topic", "/joy");

  pan_tilt_publisher_ =
    this->create_publisher<image_recognition_msgs::msg::PanTiltMsgs>(control_topic.c_str(), 10);
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic.c_str(), 1, std::bind(&Joy::joy_callback, this, std::placeholders::_1));
  status_subscriber_ = this->create_subscription<image_recognition_msgs::msg::PanTiltStatusMsgs>(
    status_topic, 1, std::bind(&Joy::status_callback, this, std::placeholders::_1));
}

void Joy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->axes.size() > std::max(0, 4)) {
    auto pan_tilt_msg = image_recognition_msgs::msg::PanTiltMsgs();
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Clock().now();
    header.frame_id = frame_id;

    pan += static_cast<int32_t>(msg->axes[0] * steps[0]);
    tilt += static_cast<int32_t>(msg->axes[4] * steps[1]);

    pan = std::clamp(pan, pan_range[0], pan_range[1]);
    tilt = std::clamp(tilt, tilt_range[0], tilt_range[1]);

    pan_tilt_msg.pan = pan;
    pan_tilt_msg.tilt = tilt;

    pan_tilt_publisher_->publish(pan_tilt_msg);

  } else {
    RCLCPP_WARN(this->get_logger(), "Joystick axes size is smaller than expected!");
  }
}

void Joy::status_callback(const image_recognition_msgs::msg::PanTiltStatusMsgs::SharedPtr msg)
{
  if (!is_init) {
    pan = msg->pan;
    tilt = msg->tilt;
    is_init = true;
  }
  frame_id = msg->header.frame_id;
  pan_range[0] = msg->pan_range[0];
  pan_range[1] = msg->pan_range[1];
  tilt_range[0] = msg->tilt_range[0];
  tilt_range[1] = msg->tilt_range[1];
  steps[0] = msg->steps[0] * 2;
  steps[1] = msg->steps[1] * 2;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Joy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
