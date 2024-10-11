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

#include "../include/usb_camera/usb_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

UsbCameraNode::UsbCameraNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("usb_camera_node", options)
{
  RCLCPP_INFO(this->get_logger(), "UsbCameraNode created.");
}

UsbCameraNode::~UsbCameraNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  if (camera_) {
    camera_->stop_stream();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsbCameraNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring UsbCameraNode Lifecycle...");

  camera_ = std::make_shared<usb_cam>();

  get_param();
  int width, height;
  sscanf(resolution_str.c_str(), "%dx%d", &width, &height);

  m_deviceInfo device_info = camera_->get_device_info(camera_path);
  if (device_info.device_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to retrieve device info for %s", camera_path.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  bool format_supported = false;
  bool resolution_supported = false;
  bool fps_supported = false;

  std::pair<int, int> highest_resolution = {0, 0};
  float highest_fps = 0.0;
  std::string highest_format;

  std::vector<std::pair<int, int>> supported_resolutions;
  std::vector<float> supported_fps_list;
  std::vector<std::string> supported_formats;

  for (const auto & supported_format : device_info.formats) {
    supported_formats.push_back(supported_format);
    if (camera_->is_format_supported(supported_format, format)) {
      format_supported = true;

      for (const auto & res_info : device_info.resolution_info) {
        supported_resolutions.push_back(res_info.resolution);

        if (
          res_info.resolution.first * res_info.resolution.second >
          highest_resolution.first * highest_resolution.second) {
          highest_resolution = res_info.resolution;
        }

        for (float supported_fps : res_info.fps) {
          supported_fps_list.push_back(supported_fps);

          if (supported_fps > highest_fps) {
            highest_fps = supported_fps;
          }

          if (res_info.resolution.first == width && res_info.resolution.second == height) {
            resolution_supported = true;
            if (supported_fps == fps) {
              fps_supported = true;
            }
          }
        }
      }
    }

    if (supported_format != highest_format) {
      highest_format = supported_format;
    }
  }

  if (!format_supported || !resolution_supported || !fps_supported) {
    RCLCPP_WARN(this->get_logger(), "Requested configuration is not supported.");

    RCLCPP_INFO(this->get_logger(), "Supported resolutions:");
    for (const auto & res : supported_resolutions) {
      RCLCPP_INFO(this->get_logger(), "%dx%d", res.first, res.second);
    }

    RCLCPP_INFO(this->get_logger(), "Supported FPS:");
    for (float supported_fps : supported_fps_list) {
      RCLCPP_INFO(this->get_logger(), "%.2f", supported_fps);
    }

    RCLCPP_INFO(this->get_logger(), "Supported formats:");
    for (const auto & supported_format : supported_formats) {
      RCLCPP_INFO(this->get_logger(), "%s", supported_format.c_str());
    }

    if (!format_supported) {
      format = highest_format;
      RCLCPP_WARN(this->get_logger(), "Using highest supported format: %s", format.c_str());
      this->set_parameters({rclcpp::Parameter("format", format)});
    }
    if (!resolution_supported) {
      width = highest_resolution.first;
      height = highest_resolution.second;
      resolution_str = std::to_string(width) + "x" + std::to_string(height);
      RCLCPP_WARN(this->get_logger(), "Using highest supported resolution: %dx%d", width, height);
      this->set_parameters({rclcpp::Parameter("resolution", resolution_str)});
    }
    if (!fps_supported) {
      fps = highest_fps;
      RCLCPP_WARN(this->get_logger(), "Using highest supported FPS: %.2f", fps);
      this->set_parameters({rclcpp::Parameter("fps", fps)});
    }
  }

  m_deviceConfig config;
  config.path = camera_path;
  config.resolution = std::make_pair(width, height);
  config.fps = fps;
  config.format = format;

  image_width = width;
  image_height = height;

  v4l2_stream_err stream_result = camera_->start_stream(config);
  if (stream_result != STREAM_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start camera stream: error code %d", stream_result);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic.c_str(), 10);
  compressed_image_pub_ =
    this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic.c_str(), 10);
  compressed_depth_pub_ =
    this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_depth_topic.c_str(), 10);
  camera_info_pub_ =
    this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic.c_str(), 10);

  set_camera();

  this->param_change_callback_ = this->add_on_set_parameters_callback(
    std::bind(&UsbCameraNode::on_parameter_change, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsbCameraNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating UsbCameraNode Lifecycle...");

  image_pub_->on_activate();
  compressed_image_pub_->on_activate();
  compressed_depth_pub_->on_activate();
  camera_info_pub_->on_activate();

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / (int)fps), [this]() {
    if (camera_ && camera_->streaming) {
      cv::Mat frame = camera_->m_image;
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = frame_id;
      if (!frame.empty()) {
        sensor_msgs::msg::Image::SharedPtr img_msg =
          cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*img_msg);

        sensor_msgs::msg::CompressedImage::SharedPtr compressed_img_msg =
          std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_img_msg->header = header;
        compressed_img_msg->format = "jpeg";
        cv::imencode(".jpg", frame, compressed_img_msg->data);
        compressed_image_pub_->publish(*compressed_img_msg);

        sensor_msgs::msg::CompressedImage::SharedPtr compressed_depth_msg =
          std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_depth_msg->header = header;
        compressed_depth_msg->format = "png";
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::imencode(".png", gray_frame, compressed_depth_msg->data);
        compressed_depth_pub_->publish(*compressed_depth_msg);
      }
      publish_camera_info(header);
    }
  });

  RCLCPP_INFO(this->get_logger(), "UsbCameraNode activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsbCameraNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating UsbCameraNode...");

  timer_.reset();

  image_pub_->on_deactivate();
  compressed_image_pub_->on_deactivate();
  compressed_depth_pub_->on_deactivate();
  camera_info_pub_->on_deactivate();

  if (camera_) {
    camera_->stop_stream();
  }

  RCLCPP_INFO(this->get_logger(), "UsbCameraNode deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn UsbCameraNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up UsbCameraNode...");

  if (camera_) {
    camera_.reset();
  }
  image_pub_.reset();
  compressed_image_pub_.reset();
  compressed_depth_pub_.reset();
  camera_info_pub_.reset();

  RCLCPP_INFO(this->get_logger(), "UsbCameraNode cleaned up.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
UsbCameraNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down UsbCameraNode from state: %s", state.label().c_str());

  if (camera_) {
    camera_->stop_stream();
    camera_.reset();
  }
  image_pub_.reset();
  compressed_image_pub_.reset();
  compressed_depth_pub_.reset();
  camera_info_pub_.reset();

  RCLCPP_INFO(this->get_logger(), "UsbCameraNode shut down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult UsbCameraNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "brightness") {
      brightness = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing brightness to: %d", brightness);
      set_param("brightness", brightness);
    } else if (param.get_name() == "contrast") {
      contrast = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing contrast to: %d", contrast);
      set_param("contrast", contrast);
    } else if (param.get_name() == "saturation") {
      saturation = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing saturation to: %d", saturation);
      set_param("saturation", saturation);
    } else if (param.get_name() == "hue") {
      hue = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing hue to: %d", hue);
      set_param("hue", hue);
    } else if (param.get_name() == "gamma") {
      gamma = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing gamma to: %d", gamma);
      set_param("gamma", gamma);
    } else if (param.get_name() == "sharpness") {
      sharpness = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing sharpness to: %d", sharpness);
      set_param("sharpness", sharpness);
    } else if (param.get_name() == "whitebalance") {
      whitebalance = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing whitebalance to: %d", whitebalance);
      set_param("whitebalance", whitebalance);
    } else if (param.get_name() == "auto_whitebalance") {
      auto_whitebalance = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing auto_whitebalance to: %d", auto_whitebalance);
      set_param_auto("whitebalance", "auto_whitebalance", whitebalance, auto_whitebalance);
    } else if (param.get_name() == "exposure") {
      exposure = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing exposure to: %d", exposure);
      set_param("exposure", exposure);
    } else if (param.get_name() == "auto_exposure") {
      auto_exposure = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing auto_exposure to: %d", auto_exposure);
      set_param_auto("exposure", "auto_exposure", exposure, auto_exposure);
    } else if (param.get_name() == "focus") {
      focus = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing focus to: %d", focus);
      set_param("focus", focus);
    } else if (param.get_name() == "auto_focus") {
      auto_focus = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing auto_focus to: %d", auto_focus);
      set_param_auto("focus", "auto_focus", focus, auto_focus);
    } else if (param.get_name() == "zoom") {
      zoom = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing zoom to: %d", zoom);
      set_param("zoom", zoom);
    } else if (param.get_name() == "pan") {
      pan = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing pan to: %d", pan);
      set_param("pan", pan);
    } else if (param.get_name() == "tilt") {
      tilt = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing tilt to: %d", tilt);
      set_param("tilt", tilt);
    } else if (param.get_name() == "rotate") {
      rotate = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Changing rotate to: %d", rotate);
      set_param("rotate", rotate);
    } else if (param.get_name() == "horizontal_flip") {
      horizontal_flip = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing horizontal_flip to: %d", horizontal_flip);
      set_param("horizontal_flip", horizontal_flip);
    } else if (param.get_name() == "vertical_flip") {
      vertical_flip = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Changing vertical_flip to: %d", vertical_flip);
      set_param("vertical_flip", vertical_flip);
    }
  }

  return result;
}

void UsbCameraNode::get_param()
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
  camera_path = this->declare_parameter<std::string>("device", "/dev/video0");
  frame_id = this->declare_parameter<std::string>("frame_id", "camera");
  resolution_str = this->declare_parameter<std::string>("resolution", "640x480");
  fps = this->declare_parameter<double>("fps", 30.0);
  format = this->declare_parameter<std::string>("format", "MJPEG");
  brightness = this->declare_parameter<int>("brightness", -1);
  contrast = this->declare_parameter<int>("contrast", -1);
  saturation = this->declare_parameter<int>("saturation", -1);
  hue = this->declare_parameter<int>("hue", -1);
  gamma = this->declare_parameter<int>("gamma", -1);
  sharpness = this->declare_parameter<int>("sharpness", -1);
  whitebalance = this->declare_parameter<int>("whitebalance", -1);
  auto_whitebalance = this->declare_parameter<bool>("auto_whitebalance", true);
  exposure = this->declare_parameter<int>("exposure", -1);
  auto_exposure = this->declare_parameter<bool>("auto_exposure", true);
  focus = this->declare_parameter<int>("focus", -1);
  auto_focus = this->declare_parameter<bool>("auto_focus", true);
  zoom = this->declare_parameter<int>("zoom", -1);
  pan = this->declare_parameter<int>("pan", -1);
  tilt = this->declare_parameter<int>("tilt", -1);
  rotate = this->declare_parameter<int>("rotate", -1);
  horizontal_flip = this->declare_parameter<bool>("horizontal_flip", false);
  vertical_flip = this->declare_parameter<bool>("vertical_flip", false);

  // Camera Information Configuration
  // Camera matrix
  int camera_matrix_rows = this->declare_parameter<int>("camera_matrix.rows", 3);
  int camera_matrix_cols = this->declare_parameter<int>("camera_matrix.cols", 3);
  camera_matrix = this->declare_parameter<std::vector<double>>(
    "camera_matrix.data", std::vector<double>(camera_matrix_rows * camera_matrix_cols, 0.0));

  // Distortion coefficients
  int distortion_coeff_rows = this->declare_parameter<int>("distortion_coefficients.rows", 1);
  int distortion_coeff_cols = this->declare_parameter<int>("distortion_coefficients.cols", 5);
  distortion_coeffs = this->declare_parameter<std::vector<double>>(
    "distortion_coefficients.data",
    std::vector<double>(distortion_coeff_rows * distortion_coeff_cols, 0.0));

  // Rectification matrix
  int rectification_matrix_rows = this->declare_parameter<int>("rectification_matrix.rows", 3);
  int rectification_matrix_cols = this->declare_parameter<int>("rectification_matrix.cols", 3);
  rectification_matrix = this->declare_parameter<std::vector<double>>(
    "rectification_matrix.data",
    std::vector<double>(rectification_matrix_rows * rectification_matrix_cols, 0.0));

  // Projection matrix
  int projection_matrix_rows = this->declare_parameter<int>("projection_matrix.rows", 3);
  int projection_matrix_cols = this->declare_parameter<int>("projection_matrix.cols", 4);
  projection_matrix = this->declare_parameter<std::vector<double>>(
    "projection_matrix.data",
    std::vector<double>(projection_matrix_rows * projection_matrix_cols, 0.0));

  distortion_model = this->declare_parameter<std::string>("distortion_model", "plumb_bob");
}

void UsbCameraNode::set_param(std::string parameter_name, int parameter_value)
{
  v4l2_queryctrl queryctrl;

  int control_id = camera_->get_control_by_name(parameter_name);

  if (control_id == -1) {
    RCLCPP_WARN(this->get_logger(), "Unknown control: %s", parameter_name.c_str());
    return;
  }

  //   RCLCPP_INFO(
  //     this->get_logger(), "Setting control: %s (ID: %d) with value: %d",
  //     parameter_name.c_str(), control_id, parameter_value);

  if (parameter_value != -1) {
    if (camera_->query_control(control_id, queryctrl)) {
      if (camera_->set_control(control_id, parameter_value)) {
        RCLCPP_INFO(
          this->get_logger(), "Successfully set %s to %d", parameter_name.c_str(), parameter_value);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Failed to set %s to %d", parameter_name.c_str(), parameter_value);
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(), "%s is not available on this device.", parameter_name.c_str());
    }
  } else {
    int current_value = camera_->get_control(control_id);
    if (current_value != -1) {
      RCLCPP_INFO(this->get_logger(), "%s : %d", parameter_name.c_str(), current_value);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "%s is not available on this device.", parameter_name.c_str());
    }
  }
}

void UsbCameraNode::set_param_auto(
  std::string parameter_name, std::string auto_parameter_name, int parameter_value,
  bool auto_parameter_value)
{
  if (!auto_parameter_value) {
    set_param(parameter_name, parameter_value);
  } else {
    set_param(auto_parameter_name, auto_parameter_value);
  }
}

void UsbCameraNode::set_camera()
{
  std::cout << std::endl;
  RCLCPP_INFO(this->get_logger(), "UsbCameraNode Lifecycle configured.");
  RCLCPP_INFO(this->get_logger(), "Topic : %s", topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Compressed Image Topic : %s", compressed_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Compressed Depth Topic : %s", compressed_depth_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Camera Info Topic : %s", camera_info_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Device : %s", camera_path.c_str());
  RCLCPP_INFO(this->get_logger(), "Frame ID : %s", frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "Resolution : %s", resolution_str.c_str());
  RCLCPP_INFO(this->get_logger(), "FPS : %f", fps);
  RCLCPP_INFO(this->get_logger(), "Format : %s", format.c_str());

  set_param("brightness", brightness);
  set_param("contrast", contrast);
  set_param("saturation", saturation);
  set_param("hue", hue);
  set_param("gamma", gamma);
  set_param("sharpness", sharpness);
  set_param_auto("whitebalance", "auto_whitebalance", whitebalance, auto_whitebalance);
  set_param_auto("exposure", "auto_exposure", exposure, auto_exposure);
  set_param_auto("focus", "auto_focus", focus, auto_focus);
  set_param("zoom", zoom);
  set_param("pan", pan);
  set_param("tilt", tilt);
  set_param("rotate", rotate);
  set_param("horizontal_flip", horizontal_flip);
  set_param("vertical_flip", vertical_flip);
  std::cout << std::endl;
}

void UsbCameraNode::publish_camera_info(const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header = header;
  camera_info_msg.distortion_model = distortion_model;
  camera_info_msg.d.resize(distortion_coeffs.size());
  std::copy(camera_matrix.begin(), camera_matrix.end(), camera_info_msg.k.begin());
  std::copy(distortion_coeffs.begin(), distortion_coeffs.end(), camera_info_msg.d.begin());
  std::copy(rectification_matrix.begin(), rectification_matrix.end(), camera_info_msg.r.begin());
  std::copy(projection_matrix.begin(), projection_matrix.end(), camera_info_msg.p.begin());
  camera_info_msg.width = image_width;
  camera_info_msg.height = image_height;
  camera_info_pub_->publish(camera_info_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UsbCameraNode>();

  auto configure_result =
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure UsbCameraNode.");
    return 1;
  }

  auto activate_result =
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate UsbCameraNode.");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}