#pragma once

#include "enum/armor_id.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::data {
struct ArmorImageSpacing {
    enumeration::ArmorId id = enumeration::ArmorId::Unknow;
    /// 以ROS系，光轴方向为x, 上为z,
    /// 四个点为： 1->左上 2->右上 3->右下 4->左下,
    /// 点实际坐标为opencv系下的像素坐标
    cv::Point2f image_points[4] = { {}, {}, {}, {} };
};
}