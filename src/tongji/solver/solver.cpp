#include "solver.hpp"

#include <chrono>
#include <memory>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <yaml-cpp/yaml.h>

#include "data/armor_camera_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "solved_armor.hpp"
#include "tongji/solver/reprojection_util.hpp"
#include "util/coordinate.hpp"
#include "util/index.hpp"
#include "util/math.hpp"
namespace world_exe::tongji::solver {

class Solver::Impl {
public:
    Impl(const std::string& config_path) {
        const auto yaml           = YAML::LoadFile(config_path);
        auto R_gimbalcamera_data  = yaml["R_gimbal2camera"].as<std::vector<double>>();
        auto t_gimbal2camera_data = yaml["t_gimbal2camera"].as<std::vector<double>>();
        Eigen::Matrix3d R_muzzle2camera_ =
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbalcamera_data.data());
        t_gimbal2camera_ = Eigen::Matrix<double, 3, 1>(t_gimbal2camera_data.data());

        reprojection_util_ = std::make_unique<ReprojectionUtil>(R_gimbal2camera_, t_gimbal2camera_);
    }

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> EstimateAllArmorPoses(
        std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
        std::vector<data::ArmorCameraSpacing> armor_plates;

        for (int i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); i++) {
            const auto& armor_id = util::enumeration::GetArmorIdFlag(i);
            const auto& armors   = armors_in_image->GetArmors(armor_id);

            for (const auto& armor : armors) {
                auto solved_armor = EstimatePose(armor);
                armor_plates.emplace_back(solved_armor);
            }
        }
        return std::make_shared<SolvedArmor>(armor_plates);
    }

    data::ArmorCameraSpacing EstimatePose(
        const world_exe::data::ArmorImageSpacing& armor_in_image) const {
        const auto result = EstimatePnp(armor_in_image);

        data::ArmorCameraSpacing pose;
        pose.id          = armor_in_image.id;
        pose.orientation = Eigen::Quaterniond(result.R_armor2camera).normalized();
        pose.position    = result.xyz_in_camera;
        return pose;
    }

    auto OptimizeYawByReprojection(const data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
        const double& initial_armor_yaw_in_gimbal) const -> const double {
        constexpr double SEARCH_RANGE = 140; // degree
        const auto yaw0 = util::math::clamp_pm_pi(gimbal_yaw - SEARCH_RANGE / 2 * CV_PI / 180.0);

        auto min_error = 1e10;
        auto best_yaw  = initial_armor_yaw_in_gimbal;

        for (int i = 0; i < SEARCH_RANGE; i++) {
            double yaw = util::math::clamp_pm_pi(yaw0 + i * CV_PI / 180.0);

            auto error = reprojection_util_->CalculateYawError(
                armor_in_image, armor_xyz_in_gimbal, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

            if (error < min_error) {
                min_error = error;
                best_yaw  = yaw;
            }
        }
        return best_yaw;
    }

    const std::time_t GetTimeStamp() const {
        return time_stamp::TimeStamp(std::chrono::steady_clock::now()).GetTimeStamp();
    }

private:
    auto EstimatePnp(const world_exe::data::ArmorImageSpacing& armor_in_image) const
        -> const PnPResultInGimbal {
        const auto& object_points = armor_in_image.isLargeArmor
            ? parameters::Robomaster::LargeArmorObjectPointsOpencv
            : parameters::Robomaster::NormalArmorObjectPointsOpencv;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points, armor_in_image.image_points,
            parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), rvec, tvec, false,
            cv::SOLVEPNP_IPPE);

        // 1. P_C_cv -> P_C_ros (位置)
        Eigen::Vector3d xyz_in_camera_cv;
        cv::cv2eigen(tvec, xyz_in_camera_cv);
        const auto xyz_in_camera = util::coordinate::opencv2ros_position(xyz_in_camera_cv);

        // 2. R_A->C_cv -> R_A->C_ros (姿态)
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera_cv;
        cv::cv2eigen(rmat, R_armor2camera_cv);
        const auto R_armor2camera = util::coordinate::opencv2ros_rotation(R_armor2camera_cv);

        Eigen::Matrix3d R_camera2gimbal = R_gimbal2camera_.transpose();
        Eigen::Vector3d t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera_;

        Eigen::Vector3d xyz_in_gimbal  = R_camera2gimbal * xyz_in_camera + t_camera2gimbal;
        Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal * R_armor2camera;

        return { xyz_in_gimbal, R_armor2gimbal, xyz_in_camera, R_armor2camera };
    }

private:
    Eigen::Matrix3d R_gimbal2camera_;
    Eigen::Vector3d t_gimbal2camera_;

    std::unique_ptr<ReprojectionUtil> reprojection_util_;
};

Solver::Solver(const std::string& config_path)
    : pimpl_(std::make_unique<Impl>(config_path)) { }
Solver::~Solver() = default;

const std::time_t Solver::GetTimeStamp() const { return pimpl_->GetTimeStamp(); }

std::shared_ptr<world_exe::interfaces::IArmorInCamera> Solver::SolvePnp(
    std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
    return pimpl_->EstimateAllArmorPoses(armors_in_image);
}

auto Solver::OptimizeYawByReprojection(const data::ArmorImageSpacing& armor_in_image,
    const Eigen::Vector3d& armor_xyz_in_gimbal, const double& gimbal_yaw,
    const double& initial_armor_yaw_in_gimbal) const -> const double {
    return OptimizeYawByReprojection(
        armor_in_image, armor_xyz_in_gimbal, gimbal_yaw, initial_armor_yaw_in_gimbal);
}

}