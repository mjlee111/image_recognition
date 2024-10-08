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

#include "../include/usb_camera/usb_camera_viewer.hpp"

UsbCameraViewer::UsbCameraViewer() : Node("usb_camera_viewer")
{
  image_topic = this->declare_parameter<std::string>("image_topic", "/camera/image_raw");

  image_topic = this->get_parameter("image_topic").as_string();

  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 10, std::bind(&UsbCameraViewer::image_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "UsbCameraViewer node has been started.");
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", image_topic.c_str());
}

void UsbCameraViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static cv::Mat frame;  // memory leak
  try {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty frame.");
      return;
    }

    cv::imshow(image_topic.c_str(), frame);
    cv::waitKey(1);

    if (cv::waitKey(10) == 27) {
      rclcpp::shutdown();
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<UsbCameraViewer>());

  rclcpp::shutdown();
  return 0;
}
