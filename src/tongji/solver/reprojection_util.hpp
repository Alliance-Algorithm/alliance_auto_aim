#pragma once

#include <cstdlib>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "util/coordinate.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::solver {

class ReprojectionUtil {
public:
    explicit ReprojectionUtil(
        const Eigen::Matrix3d& R_gimbal2camera, const Eigen::Vector3d& t_gimbal2camera)
        : R_gimbal2camera_(R_gimbal2camera)
        , t_gimbal2camera_(t_gimbal2camera) { }

    std::vector<cv::Point2d> ReprojectWithYpr(const Eigen::Vector3d& xyz_in_gimbal,
        const double& yaw, const double& pitch, const bool& is_large) const {

        double sin_yaw = std::sin(yaw);
        double cos_yaw = std::cos(yaw);

        double sin_pitch = std::sin(pitch);
        double cos_pitch = std::cos(pitch);

        // clang-format off
        const Eigen::Matrix3d R_armor2gimbal {
          {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
          {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
          {         -sin_pitch,        0,           cos_pitch}
        };
        // clang-format on

        return ReprojectWithR_gimbal(xyz_in_gimbal, R_armor2gimbal, is_large);
    }

    double CalculateYawError(const world_exe::data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& armor_yaw_in_gimbal,
        const double& inclined) const {
        auto pitch =
            (armor_in_image.id == enumeration::ArmorIdFlag::Outpost) ? -15.0 * CV_PI / 180.0 : 15.0;

        auto image_points = ReprojectWithYpr(
            armor_xyz_in_gimbal, armor_yaw_in_gimbal, pitch, armor_in_image.isLargeArmor);

        auto error = 0.0;
        for (int i = 0; i < 4; i++) {
            error += cv::norm(armor_in_image.image_points[i] - image_points[i]);
        }

        // SJTU_cost
        // auto cost = SJTU_cost(image_points, armor_in_image.image_points, inclined);
        return error;
    }

    double CalculatePitchError(const world_exe::data::ArmorImageSpacing& armor_in_image,
        const double& armor_yaw_in_gimbal, const Eigen::Vector3d& armor_xyz_in_gimbal,
        const double& pitch) const {

        auto image_points = ReprojectWithYpr(
            armor_xyz_in_gimbal, armor_yaw_in_gimbal, pitch, armor_in_image.isLargeArmor);

        auto error = 0.0;
        for (int i = 0; i < 4; i++) {
            error += cv::norm(armor_in_image.image_points[i] - image_points[i]);
        }

        return error;
    }

private:
    auto TransformGimbalToCamera(const Eigen::Matrix3d& R_armor2gimbal,
        const Eigen::Vector3d& xyz_in_gimbal, cv::Vec3d& rvec_out, cv::Vec3d& tvec_out) const
        -> void {

        // R_A->C = R_G->C * R_A->G
        Eigen::Matrix3d R_armor2camera = R_gimbal2camera_ * R_armor2gimbal;
        // t_A->C = R_G->C * P_G + t_G->C
        Eigen::Vector3d t_armor2camera = R_gimbal2camera_ * xyz_in_gimbal + t_gimbal2camera_;

        cv::Mat R_armor2camera_cv;
        cv::eigen2cv(util::coordinate::ros2opencv_rotation(R_armor2camera), R_armor2camera_cv);
        cv::Rodrigues(R_armor2camera_cv, rvec_out);

        const auto t_armor2camera_cv = util::coordinate::ros2opencv_position(t_armor2camera);
        tvec_out = cv::Vec3d(t_armor2camera_cv[0], t_armor2camera_cv[1], t_armor2camera_cv[2]);
    }

    std::vector<cv::Point2d> ReprojectWithR_gimbal(const Eigen::Vector3d& armor_xyz_in_gimbal,
        const Eigen::Matrix3d& R_armor2gimbal, const bool& is_large) const {

        cv::Vec3d rvec, tvec;
        TransformGimbalToCamera(R_armor2gimbal, armor_xyz_in_gimbal, rvec, tvec);

        std::vector<cv::Point2d> image_points;
        const auto& object_points = (is_large)
            ? parameters::Robomaster::LargeArmorObjectPointsOpencv
            : parameters::Robomaster::NormalArmorObjectPointsOpencv;
        cv::projectPoints(object_points, rvec, tvec,
            parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), image_points);
        return image_points;
    }

    double SJTU_cost(const std::vector<cv::Point2d>& cv_refs,
        const std::vector<cv::Point2d>& cv_pts, const double& inclined) const {
        std::size_t size = cv_refs.size();
        std::vector<Eigen::Vector2d> refs;
        std::vector<Eigen::Vector2d> pts;
        for (std::size_t i = 0u; i < size; ++i) {
            refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
            pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
        }
        double cost = 0.;
        for (std::size_t i = 0u; i < size; ++i) {
            std::size_t p = (i + 1u) % size;
            // i - p 构成线段。过程：先移动起点，再补长度，再旋转
            Eigen::Vector2d ref_d = refs[p] - refs[i]; // 标准
            Eigen::Vector2d pt_d  = pts[p] - pts[i];
            // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
            double pixel_dis = // dis 是指方差平面内到原点的距离
                (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm())
                    + std::fabs(ref_d.norm() - pt_d.norm()))
                / ref_d.norm();
            double angular_dis =
                ref_d.norm() * util::math::get_abs_angle(ref_d, pt_d) / ref_d.norm();
            // 平方可能是为了配合 sin 和 cos
            // 弧度差代价（0 度左右占比应该大）
            double cost_i = util::math::square(pixel_dis * std::sin(inclined))
                + util::math::square(angular_dis * std::cos(inclined))
                    * 2.0; // DETECTOR_ERROR_PIXEL_BY_SLOPE
            // 重投影像素误差越大，越相信斜率
            cost += std::sqrt(cost_i);
        }
        return cost;
    }

    Eigen::Matrix3d R_gimbal2camera_;
    Eigen::Vector3d t_gimbal2camera_;
};
}
