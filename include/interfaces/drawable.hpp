#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {
class IDrawable {
    virtual void Draw(cv::InputArray, cv::OutputArray);
    virtual void Draw(cv::InputOutputArray);
};
}