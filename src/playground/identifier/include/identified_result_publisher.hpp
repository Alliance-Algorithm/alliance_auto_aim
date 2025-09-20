#ifndef IDENTIFIED_RESULT_PUBLISHER_HPP_
#define IDENTIFIED_RESULT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <identifier/msg/point_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "armor_identifier.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;
class IdentifiedResultPublisher : public rclcpp::Node
{
public:
  IdentifiedResultPublisher();

  void update_frame(const cv::Mat& frame); 
  void publish_message();
  void publish_image();
  void publish_markers();
  void show_ui(); 

private:
  rclcpp::Publisher<identifier::msg::PointData>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  ArmorIdentifier armor_identifier_;
  std::vector<ArmorCoordinates> armor_identifiers_;
};

#endif  // IDENTIFIED_RESULT_PUBLISHER_HPP_
