#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
namespace world_exe::parameters {
class HikCameraProfile {
public:
    HikCameraProfile(const double& fx, const double& fy, const double& cx, const double& cy,
        const double& k1, const double& k2, const double& k3);

    inline static void set_width_height(const int& width, const int& height);
    inline static const cv::Mat& get_intrinsic_parameters();
    inline static const cv::Mat& get_distortion_parameters();
    inline static const std::tuple<int, int>& get_width_height();

private:
    struct Impl;

    static std::unique_ptr<Impl> impl_;
};
}