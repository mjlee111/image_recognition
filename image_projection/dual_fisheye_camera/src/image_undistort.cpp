#include "../include/dual_fisheye_camera/image_undistort.hpp"

ImageUndistorter::ImageUndistorter() : Node("image_undistorter")
{
  get_param();

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_.c_str(), 10,
    std::bind(&ImageUndistorter::imageCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(calibrated_image_topic_.c_str(), 10);
  RCLCPP_INFO(this->get_logger(), "Image Undistorter Initialized");
}

void ImageUndistorter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::Mat undistorted;
    cv::undistort(cv_ptr->image, undistorted, camera_matrix_, dist_coeffs_, new_camera_matrix_);
    sensor_msgs::msg::Image::SharedPtr undistorted_msg =
      cv_bridge::CvImage(msg->header, msg->encoding, undistorted).toImageMsg();
    publisher_->publish(*undistorted_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageUndistorter::get_param()
{
  image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
  calibrated_image_topic_ = image_topic_ + "/calibrated";
  image_width_ = this->declare_parameter<int>("image_width", 736);
  image_height_ = this->declare_parameter<int>("image_height", 736);
  camera_matrix_data_ = this->declare_parameter<std::vector<double>>(
    "camera_matrix",
    {230.835588, -0.104723, 366.729170, 0.0, 232.336174, 365.053040, 0.0, 0.0, 1.0});
  dist_coeffs_data_ = this->declare_parameter<std::vector<double>>(
    "dist_coeffs", {-0.020724, -0.004885, 0.002493, -0.000835});
  rectification_matrix_data_ = this->declare_parameter<std::vector<double>>(
    "rectification_matrix", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  projection_matrix_data_ = this->declare_parameter<std::vector<double>>(
    "projection_matrix",
    {230.835588, -0.104723, 366.729170, 0.0, 0.0, 232.336174, 365.053040, 0.0, 0.0, 0.0, 1.0, 0.0});

  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data_.data());
  dist_coeffs_ = cv::Mat(1, 4, CV_64F, dist_coeffs_data_.data());
  rectification_matrix_ = cv::Mat(3, 3, CV_64F, rectification_matrix_data_.data());
  projection_matrix_ = cv::Mat(3, 4, CV_64F, projection_matrix_data_.data());

  new_camera_matrix_ = cv::getOptimalNewCameraMatrix(
    camera_matrix_, dist_coeffs_, cv::Size(image_width_, image_height_), 1.0);

  RCLCPP_INFO(this->get_logger(), "Image Topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Calibrated Image Topic: %s", calibrated_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Image Width: %d", image_width_);
  RCLCPP_INFO(this->get_logger(), "Image Height: %d", image_height_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageUndistorter>());
  rclcpp::shutdown();
  return 0;
}