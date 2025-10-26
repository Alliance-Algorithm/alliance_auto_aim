#include "solver.hpp"

#include <chrono>
#include <memory>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "data/armor_camera_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "solved_armor.hpp"

#include "util/coordinate.hpp"
#include "util/index.hpp"
#include "util/math.hpp"
namespace world_exe::tongji::solver {

class SolverImpl {
public:
    SolverImpl(const std::string& config_path) {
        const auto yaml           = YAML::LoadFile(config_path);
        auto R_gimbalcamera_data  = yaml["R_gimbal2camera"].as<std::vector<double>>();
        auto t_gimbal2camera_data = yaml["t_gimbal2camera"].as<std::vector<double>>();
        Eigen::Matrix3d R_muzzle2camera_ =
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbalcamera_data.data());
        t_gimbal2camera_ = Eigen::Matrix<double, 3, 1>(t_gimbal2camera_data.data());
    }

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
        std::vector<data::ArmorCameraSpacing> armor_plates;

        for (int i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); i++) {
            const auto& armor_id = util::enumeration::GetArmorIdFlag(i);
            const auto& armors   = armors_in_image->GetArmors(armor_id);
            for (const auto& armor : armors) {
                auto solved_armor = Solve(armor);
                armor_plates.emplace_back(solved_armor);
            }
        }
        return std::make_shared<SolvedArmor>(armor_plates);
    }

    world_exe::data::ArmorCameraSpacing Solve(
        const world_exe::data::ArmorImageSpacing& armor_in_image) const {
        const auto& object_points = armor_in_image.isLargeArmor
            ? parameters::Robomaster::LargeArmorObjectPointsOpencv
            : parameters::Robomaster::NormalArmorObjectPointsOpencv;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points, armor_in_image.image_points,
            parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), rvec, tvec, false,
            cv::SOLVEPNP_IPPE);

        Eigen::Vector3d xyz_in_camera;
        cv::cv2eigen(tvec, xyz_in_camera);

        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(rmat, R_armor2camera);

        const auto armor_in_camera =
            convert_to_ros_spacing(armor_in_image.id, xyz_in_camera, R_armor2camera);
        return armor_in_camera;
    }

    auto OptimizeYaw(const data::ArmorImageSpacing& armor_in_image, const double& gimbal_yaw,
        const double& armor_yaw_in_gimbal, const Eigen::Vector3d& armor_xyz_in_gimbal) const
        -> const double {

        constexpr double SEARCH_RANGE = 140; // degree

        const auto yaw0 = util::math::clamp_pm_pi(gimbal_yaw - SEARCH_RANGE / 2 * CV_PI / 180.0);

        auto min_error = 1e10;
        auto best_yaw  = armor_yaw_in_gimbal;
        for (int i = 0; i < SEARCH_RANGE; i++) {
            double yaw = util::math::clamp_pm_pi(yaw0 + i * CV_PI / 180.0);
            auto error = ArmorReprojectionError(
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
    std::vector<cv::Point2d> ReprojectArmor(const Eigen::Vector3d& xyz_in_gimbal, const double& yaw,
        const bool& is_large, const enumeration::ArmorIdFlag& id) const {
        auto sin_yaw = std::sin(yaw);
        auto cos_yaw = std::cos(yaw);

        auto pitch     = (id == enumeration::ArmorIdFlag::Outpost) ? -15.0 * CV_PI / 180.0
                                                                   : 15.0 * CV_PI / 180.0;
        auto sin_pitch = std::sin(pitch);
        auto cos_pitch = std::cos(pitch);

        // clang-format off
        const Eigen::Matrix3d R_armor2gimbal {
          {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
          {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
          {         -sin_pitch,        0,           cos_pitch}
  };
        // clang-format on

        // get R_armor2camera t_armor2camera
        Eigen::Matrix3d R_armor2camera = R_gimbal2camera_ * R_armor2gimbal;
        Eigen::Vector3d t_armor2camera = (R_gimbal2camera_ * xyz_in_gimbal + t_gimbal2camera_);

        // get rvec tvec
        cv::Vec3d rvec;
        cv::Mat R_armor2camera_cv;
        cv::eigen2cv(util::coordinate::ros2opencv_rotation(R_armor2camera), R_armor2camera_cv);
        cv::Rodrigues(R_armor2camera_cv, rvec);
        const auto t_armor2camera_cv = util::coordinate::ros2opencv_position(t_armor2camera);
        cv::Vec3d tvec(t_armor2camera_cv[0], t_armor2camera_cv[1], t_armor2camera_cv[2]);

        // reproject
        std::vector<cv::Point2d> image_points;
        const auto& object_points = (is_large)
            ? parameters::Robomaster::LargeArmorObjectPointsOpencv
            : parameters::Robomaster::NormalArmorObjectPointsOpencv;
        cv::projectPoints(object_points, rvec, tvec,
            parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), image_points);
        return image_points;
    }

    double ArmorReprojectionError(const world_exe::data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_gimbal, const double& yaw,
        const double& inclined) const {
        const auto image_points = ReprojectArmor(
            armor_xyz_in_gimbal, yaw, armor_in_image.isLargeArmor, armor_in_image.id);
        auto error = 0.0;
        for (int i = 0; i < 4; i++)
            error += cv::norm(armor_in_image.image_points[i] - image_points[i]);
        // auto error = SJTU_cost(image_points, armor_in_image.image_points, inclined);

        return error;
    }

    double OutpostReprojectionError(world_exe::data::ArmorImageSpacing armor_in_image,
        Eigen::Vector3d armor_xyz_in_gimbal, const double& pitch) const {

        // solve
        const auto& object_points = (armor_in_image.isLargeArmor)
            ? world_exe::parameters::Robomaster::LargeArmorObjectPointsOpencv
            : world_exe::parameters::Robomaster::NormalArmorObjectPointsOpencv;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points, armor_in_image.image_points,
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), rvec, tvec, false,
            cv::SOLVEPNP_IPPE);

        Eigen::Vector3d xyz_in_camera_cv;
        cv::cv2eigen(tvec, xyz_in_camera_cv);
        auto xyz_in_camera = util::coordinate::opencv2ros_position(xyz_in_camera_cv);

        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera_cv;
        cv::cv2eigen(rmat, R_armor2camera_cv);
        auto R_armor2camera = util::coordinate::opencv2ros_rotation(R_armor2camera_cv);

        Eigen::Matrix3d R_armor2gimbal = R_gimbal2camera_.transpose() * R_armor2camera;
        auto armor_ypr_in_gimbal       = util::math::matrix_to_euler(R_armor2gimbal, 2, 1, 0);

        auto yaw     = armor_ypr_in_gimbal[0];
        auto sin_yaw = std::sin(yaw);
        auto cos_yaw = std::cos(yaw);

        auto sin_pitch = std::sin(pitch);
        auto cos_pitch = std::cos(pitch);

        // clang-format off
        const Eigen::Matrix3d _R_armor2gimbal {
          {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
          {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
          {         -sin_pitch,        0,           cos_pitch}};
        // clang-format on

        // get R_armor2camera t_armor2camera
        Eigen::Matrix3d _R_armor2camera = R_gimbal2camera_ * _R_armor2gimbal;
        Eigen::Vector3d t_armor2camera  = R_gimbal2camera_ * armor_xyz_in_gimbal + t_gimbal2camera_;

        // get rvec tvec
        cv::Vec3d _rvec;
        cv::Mat _R_armor2camera_cv;
        cv::eigen2cv(util::coordinate::ros2opencv_rotation(_R_armor2camera), _R_armor2camera_cv);
        cv::Rodrigues(_R_armor2camera_cv, _rvec);
        const auto t_armor2camera_cv = util::coordinate::ros2opencv_position(t_armor2camera);
        cv::Vec3d _tvec(t_armor2camera_cv[0], t_armor2camera_cv[1], t_armor2camera_cv[2]);

        // reproject
        std::vector<cv::Point2d> image_points;
        cv::projectPoints(object_points, _rvec, _tvec,
            parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), image_points);

        auto error = 0.0;
        for (int i = 0; i < 4; i++)
            error += cv::norm(armor_in_image.image_points[i] - image_points[i]);
        return error;
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

    data::ArmorCameraSpacing convert_to_ros_spacing(const enumeration::ArmorIdFlag& id,
        const Eigen::Vector3d& xyz_in_opencv, const Eigen::Matrix3d& R_armor2camera_opencv) const {
        world_exe::data::ArmorCameraSpacing armor_in_camera;

        armor_in_camera.position = util::coordinate::opencv2ros_position(xyz_in_opencv);
        Eigen::Matrix3d R_in_ros = util::coordinate::opencv2ros_rotation(R_armor2camera_opencv);
        Eigen::Quaterniond orientation_ros(R_in_ros);
        orientation_ros.normalize();
        armor_in_camera.id          = id;
        armor_in_camera.orientation = orientation_ros;

        return armor_in_camera;
    }

private:
    Eigen::Matrix3d R_gimbal2camera_;
    Eigen::Vector3d t_gimbal2camera_;
};

Solver::Solver(const std::string& config_path)
    : pimpl_(std::make_unique<SolverImpl>(config_path)) { }
Solver::~Solver() = default;

const std::time_t Solver::GetTimeStamp() const { return pimpl_->GetTimeStamp(); }

std::shared_ptr<world_exe::interfaces::IArmorInCamera> Solver::SolvePnp(
    std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
    return pimpl_->SolvePnp(armors_in_image);
}

}