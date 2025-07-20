#pragma once

#include "interfaces/armor_in_image.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {
class IIdentifier {
public:
    const IArmorInImage identify(const cv::Mat& input_image);
};
}