#include "parameters/profile.hpp"
#include <opencv2/core/types.hpp>

namespace world_exe::parameters {

struct parameters::HikCameraProfile::Impl {
    Impl(const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3)
        : intrinsic_parameters((cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1))
        , distortion_parameters((cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3)) { }

    const cv::Mat intrinsic_parameters;
    const cv::Mat distortion_parameters;
    std::tuple<int, int> width_height;
};

HikCameraProfile::HikCameraProfile(const double& fx, const double& fy, const double& cx,
    const double& cy, const double& k1, const double& k2, const double& k3) {
    impl_ = std::make_unique<Impl>(fx, fy, cx, cy, k1, k2, k3);
}

void HikCameraProfile::set_width_height(const int& width, const int& height) {
    impl_->width_height = std::make_tuple(width, height);
}

const cv::Mat& HikCameraProfile::get_intrinsic_parameters() { return impl_->intrinsic_parameters; }
const cv::Mat& HikCameraProfile::get_distortion_parameters() {
    return impl_->distortion_parameters;
}
const std::tuple<int, int>& HikCameraProfile::get_width_height() { return impl_->width_height; }
} // namespace rmcs_auto_aim::util

std::unique_ptr<world_exe::parameters::HikCameraProfile::Impl>
    world_exe::parameters::HikCameraProfile::impl_;