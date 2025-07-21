#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {
class IDrawable {
public:
    virtual void Draw(cv::InputArray, cv::OutputArray) = 0;
    virtual void Draw(cv::InputOutputArray)            = 0;
};
}