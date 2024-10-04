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

#include "../include/dual_fisheye_camera/image_ctrl.hpp"

ImageCtrl::ImageCtrl(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("usb_camera_utils", options)
{
  RCLCPP_INFO(this->get_logger(), "ImageCtrl node has been created.");
}

ImageCtrl::~ImageCtrl()
{
  RCLCPP_INFO(this->get_logger(), "ImageCtrl node is being destroyed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImageCtrl::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring ImageCtrl...");
  get_param();

  image_pub_left_ = this->create_publisher<sensor_msgs::msg::Image>(left_topic.c_str(), 10);
  image_pub_right_ = this->create_publisher<sensor_msgs::msg::Image>(right_topic.c_str(), 10);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    topic.c_str(), 1, std::bind(&ImageCtrl::image_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Image subscriber configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImageCtrl::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating ImageCtrl...");

  image_pub_left_->on_activate();
  image_pub_right_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImageCtrl::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating ImageCtrl...");

  image_pub_left_->on_deactivate();
  image_pub_right_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImageCtrl::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up ImageCtrl...");

  image_pub_left_.reset();
  image_pub_right_.reset();
  image_sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ImageCtrl::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down ImageCtrl...");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ImageCtrl::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (image.cols != original_width || image.rows != original_height) {
      RCLCPP_ERROR(this->get_logger(), "Unexpected image size: %d x %d", image.cols, image.rows);
      return;
    }

    cv::Rect left_half(0, 0, image_width, image_height);
    cv::Rect right_half(image_width, 0, image_width, image_height);

    cv::Mat left_image = image(left_half);
    cv::Mat right_image = image(right_half);

    auto left_msg = cv_bridge::CvImage(msg->header, "bgr8", left_image).toImageMsg();
    auto right_msg = cv_bridge::CvImage(msg->header, "bgr8", right_image).toImageMsg();

    if (image_pub_left_->is_activated()) {
      image_pub_left_->publish(*left_msg);
    }

    if (image_pub_right_->is_activated()) {
      image_pub_right_->publish(*right_msg);
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageCtrl::get_param()
{
  // Camera configuration
  camera_name = this->declare_parameter<std::string>("camera_name", "camera1");
  std::string topic_str = this->declare_parameter<std::string>("topic", "/camera/image_raw");
  std::string compressed_topic_str =
    this->declare_parameter<std::string>("compressed_topic", "/camera/compressed_image");
  std::string compressed_depth_topic_str =
    this->declare_parameter<std::string>("compressed_depth_topic", "/camera/compressed_depth");
  topic = "/" + camera_name + topic_str;
  compressed_topic = "/" + camera_name + compressed_topic_str;
  compressed_depth_topic = "/" + camera_name + compressed_depth_topic_str;
  camera_info_topic = "/" + camera_name + "/info";
  left_topic = "/" + camera_name + "/left" + topic_str;
  right_topic = "/" + camera_name + "/right" + topic_str;
  frame_id = this->declare_parameter<std::string>("frame_id", "camera");
  resolution_str = this->declare_parameter<std::string>("resolution", "640x480");
  resolution_str2 = this->declare_parameter<std::string>("split_resolution", "1504x1504");
  fps = this->declare_parameter<double>("fps", 30.0);
  format = this->declare_parameter<std::string>("format", "MJPEG");

  sscanf(resolution_str.c_str(), "%dx%d", &original_width, &original_height);
  sscanf(resolution_str2.c_str(), "%dx%d", &image_width, &image_height);

  RCLCPP_INFO(this->get_logger(), "Original Image Size : %s", resolution_str.c_str());
  RCLCPP_INFO(this->get_logger(), "Splitting to Size : %s", resolution_str2.c_str());
  RCLCPP_INFO(this->get_logger(), "Left Image Topic : %s", left_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Right Image Topic : %s", right_topic.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageCtrl>();

  auto configure_result =
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure ImageCtrl.");
    return 1;
  }

  auto activate_result =
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate ImageCtrl.");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}