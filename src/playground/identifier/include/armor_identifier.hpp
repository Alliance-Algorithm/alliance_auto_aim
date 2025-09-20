#ifndef ARMOR_IDENTIFIER_HPP_
#define ARMOR_IDENTIFIER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <identifier/msg/point_data.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct ArmorCoordinates
{
    cv::Point2f leftup={0, 0};
    cv::Point2f rightup={0, 0};
    cv::Point2f leftdown={0, 0};
    cv::Point2f rightdown={0, 0};
    cv::Point2f center={0, 0};
    bool is_identified = false;
    std::string color = "none";
};

class ArmorIdentifier
{
public:
    ArmorIdentifier();

    cv::Mat image_output;
    // 处理图像
    void  process_image(const cv::Mat& image);
    void  pair_bars(const std::vector<cv::RotatedRect>& bars,
                    const std::string & color);

    // 获取装甲板四个角点
    ArmorCoordinates get_coordinates_closest() const;//获取最靠近中心的装甲板角点
    std::vector<ArmorCoordinates> get_coordinates_all() const;//获取所有装甲板角点
    void show();

private:
    cv::Mat image_raw;
    cv::Mat image_gray;
    cv::Mat image_hsv;
    cv::Mat mask_red1;
    cv::Mat mask_red2;
    cv::Mat mask_red;
    cv::Mat mask_blue;
    std::vector<ArmorCoordinates> coordinates_;
};

#endif  // ARMOR_IDENTIFIER_HPP_
