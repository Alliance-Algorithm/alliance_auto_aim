#include "solver.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>

#include "data/armor_gimbal_control_spacing.hpp"
#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "util/math.hpp"
#include "util/transform.hpp"
namespace world_exe::tongji::solver {

class Solver::Impl {
public:
    Impl() { }
    const world_exe::data::ArmorCameraSpacing solve(
        const world_exe::data::ArmorImageSpacing& armor_in_image, Eigen::Matrix3d R_camera2gimbal,
        Eigen::Matrix3d R_gimbal2world, Eigen::Vector3d t_camera2gimbal) const {
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

        Eigen::Quaterniond orientation_in_camera(R_armor2camera);

        auto is_balance = armor_in_image.isLargeArmor
            && (armor_in_image.id == enumeration::ArmorIdFlag::InfantryIII
                || armor_in_image.id == enumeration::ArmorIdFlag::InfantryIV
                || armor_in_image.id == enumeration::ArmorIdFlag::InfantryV);

        world_exe::data::ArmorCameraSpacing armor_in_camera;
        auto orientation_ros = util::transform::opencv2ros_orientation(orientation_in_camera);
        orientation_ros.normalize();
        armor_in_camera.id          = armor_in_image.id;
        armor_in_camera.position    = util::transform::opencv2ros_position(xyz_in_camera);
        armor_in_camera.orientation = orientation_ros;

        if (!is_balance) {
            optimize_yaw(
                armor_in_image, armor_in_camera, R_camera2gimbal, R_gimbal2world, t_camera2gimbal);
        }

        return armor_in_camera;
    }

private:
    world_exe::data::ArmorGimbalControlSpacing optimize_yaw(
        const world_exe::data::ArmorImageSpacing& armor_in_image,
        const world_exe::data::ArmorCameraSpacing& armor_in_camera, Eigen::Matrix3d R_camera2gimbal,
        Eigen::Matrix3d R_gimbal2world, Eigen::Vector3d t_camera2gimbal) const {

        Eigen::Vector3d armor_in_camera_ocv =
            util::transform::ros2opencv_position(armor_in_camera.position);
        Eigen::Vector3d armor_xyz_in_gimbal =
            R_camera2gimbal * armor_in_camera_ocv + t_camera2gimbal;
        Eigen::Vector3d armor_xyz_in_world = R_gimbal2world * armor_xyz_in_gimbal; // why no t?

        Eigen::Quaterniond armor_orientation_in_camera_ocv =
            util::transform::ros2opencv_orientation(armor_in_camera.orientation);
        Eigen::Matrix3d R_armor2camera_ocv    = armor_orientation_in_camera_ocv.toRotationMatrix();
        Eigen::Matrix3d R_armor2gimbal        = R_camera2gimbal * R_armor2camera_ocv;
        Eigen::Matrix3d R_armor2world_initial = R_gimbal2world * R_armor2gimbal;

        Eigen::Vector3d armor_ypr_in_world =
            util::math::matrix_to_euler(R_armor2world_initial, 2, 1, 0);
        double initial_yaw = armor_ypr_in_world[0];

        Eigen::Vector3d gimbal_ypr = util::math::matrix_to_euler(R_gimbal2world, 2, 1, 0);

        constexpr double SEARCH_RANGE = 140; // degree
        auto yaw0 = util::math::clamp_pm_pi(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

        auto min_error = 1e10;
        auto best_yaw  = armor_ypr_in_world[0];

        for (int i = 0; i < SEARCH_RANGE; i++) {
            double yaw = util::math::clamp_pm_pi(yaw0 + i * CV_PI / 180.0);
            auto error = armor_reprojection_error(armor_in_image, R_camera2gimbal, R_gimbal2world,
                t_camera2gimbal, armor_xyz_in_world, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

            if (error < min_error) {
                min_error = error;
                best_yaw  = yaw;
            }
        }

        // armor.yaw_raw         = armor.ypr_in_world[0];
        armor_ypr_in_world[0] = best_yaw;

        Eigen::Matrix3d R_armor2world_optimized = util::math::euler_to_matrix(armor_ypr_in_world);

        Eigen::Quaterniond orientation_in_world(R_armor2world_optimized);
        orientation_in_world.normalize();

        world_exe::data::ArmorGimbalControlSpacing result;
        result.id          = armor_in_camera.id;
        result.position    = armor_xyz_in_world;
        result.orientation = orientation_in_world;
        return result;
    }

    Eigen::Vector3d camera2world(const Eigen::Vector3d& xyz_in_camera,
        Eigen::Matrix3d R_camera2world, const Eigen::Vector3d& t_camera2world) const {
        return R_camera2world * xyz_in_camera + t_camera2world;
    }

    std::vector<cv::Point2d> reproject_armor(const Eigen::Vector3d& xyz_in_world,
        Eigen::Matrix3d R_camera2gimbal, Eigen::Matrix3d R_gimbal2world,
        Eigen::Vector3d t_camera2gimbal, double yaw, bool is_large,
        enumeration::ArmorIdFlag id) const {
        auto sin_yaw = std::sin(yaw);
        auto cos_yaw = std::cos(yaw);

        auto pitch     = (id == enumeration::ArmorIdFlag::Outpost) ? -15.0 * CV_PI / 180.0
                                                                   : 15.0 * CV_PI / 180.0;
        auto sin_pitch = std::sin(pitch);
        auto cos_pitch = std::cos(pitch);

        // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
        // clang-format on

        // get R_armor2camera t_armor2camera
        const Eigen::Vector3d& t_armor2world = xyz_in_world;
        Eigen::Matrix3d R_armor2camera =
            R_camera2gimbal.transpose() * R_gimbal2world.transpose() * R_armor2world;
        Eigen::Vector3d t_armor2camera = R_camera2gimbal.transpose()
            * (R_gimbal2world.transpose() * t_armor2world - t_camera2gimbal);

        // get rvec tvec
        cv::Vec3d rvec;
        cv::Mat R_armor2camera_cv;
        cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
        cv::Rodrigues(R_armor2camera_cv, rvec);
        cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

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

    double armor_reprojection_error(world_exe::data::ArmorImageSpacing armor_in_image,
        Eigen::Matrix3d R_camera2gimbal, Eigen::Matrix3d R_gimbal2world,
        Eigen::Vector3d t_camera2gimbal, Eigen::Vector3d armor_xyz_in_world, double yaw,
        const double& inclined) const {

        auto image_points = reproject_armor(armor_xyz_in_world, R_camera2gimbal, R_gimbal2world,
            t_camera2gimbal, yaw, armor_in_image.isLargeArmor, armor_in_image.id);
        auto error        = 0.0;
        for (int i = 0; i < 4; i++)
            error += cv::norm(armor_in_image.image_points[i] - image_points[i]);
        // auto error = SJTU_cost(image_points, armor.points, inclined);

        return error;
    }

    double oupost_reprojection_error(world_exe::data::ArmorImageSpacing armor_in_image,
        Eigen::Matrix3d R_camera2gimbal, Eigen::Matrix3d R_gimbal2world,
        Eigen::Vector3d t_camera2gimbal, Eigen::Vector3d armor_xyz_in_world, const double& pitch) {

        // solve
        const auto& object_points = (armor_in_image.isLargeArmor)
            ? world_exe::parameters::Robomaster::LargeArmorObjectPointsOpencv
            : world_exe::parameters::Robomaster::NormalArmorObjectPointsOpencv;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points, armor_in_image.image_points,
            world_exe::parameters::HikCameraProfile::get_intrinsic_parameters(),
            parameters::HikCameraProfile::get_distortion_parameters(), rvec, tvec, false,
            cv::SOLVEPNP_IPPE);

        Eigen::Vector3d xyz_in_camera;
        cv::cv2eigen(tvec, xyz_in_camera);

        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(rmat, R_armor2camera);
        Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal * R_armor2camera;
        Eigen::Matrix3d R_armor2world  = R_gimbal2world * R_armor2gimbal;
        auto armor_ypr_in_gimbal       = util::math::matrix_to_euler(R_armor2gimbal, 2, 1, 0);
        auto armor_ypr_in_world        = util::math::matrix_to_euler(R_armor2world, 2, 1, 0);

        auto yaw          = armor_ypr_in_world[0];
        auto xyz_in_world = armor_xyz_in_world;

        auto sin_yaw = std::sin(yaw);
        auto cos_yaw = std::cos(yaw);

        auto sin_pitch = std::sin(pitch);
        auto cos_pitch = std::cos(pitch);

        // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
        // clang-format on

        // get R_armor2camera t_armor2camera
        const Eigen::Vector3d& t_armor2world = xyz_in_world;
        Eigen::Matrix3d _R_armor2camera =
            R_camera2gimbal.transpose() * R_gimbal2world.transpose() * _R_armor2world;
        Eigen::Vector3d t_armor2camera = R_camera2gimbal.transpose()
            * (R_gimbal2world.transpose() * t_armor2world - t_camera2gimbal);

        // get rvec tvec
        cv::Vec3d _rvec;
        cv::Mat R_armor2camera_cv;
        cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
        cv::Rodrigues(R_armor2camera_cv, _rvec);
        cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

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
};

const std::time_t& Solver::GetTimeStamp() const { return time_point_; }

}