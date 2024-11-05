#ifndef IMAGE_UNDISTORTER_HPP_
#define IMAGE_UNDISTORTER_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

class ImageUndistorter : public rclcpp::Node
{
public:
  ImageUndistorter();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  void get_param();

  int image_width_;
  int image_height_;
  std::string image_topic_, calibrated_image_topic_;
  std::vector<double> camera_matrix_data_;
  int camera_matrix_rows_, camera_matrix_cols_;
  std::vector<double> dist_coeffs_data_;
  int dist_coeffs_rows_, dist_coeffs_cols_;
  std::vector<double> rectification_matrix_data_;
  int rectification_matrix_rows_, rectification_matrix_cols_;
  std::vector<double> projection_matrix_data_;
  int projection_matrix_rows_, projection_matrix_cols_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat rectification_matrix_;
  cv::Mat projection_matrix_;
  cv::Mat new_camera_matrix_;
};

#endif