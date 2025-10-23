#include "solver.hpp"

#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "data/armor_camera_spacing.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "parameters/profile.hpp"
#include "parameters/rm_parameters.hpp"
#include "solved_armor.hpp"
#include "util/coordinate.hpp"
#include "util/index.hpp"
#include "util/math.hpp"
namespace world_exe::tongji::solver {

class SolverImpl {
public:
    SolverImpl(Eigen::Matrix3d R_camera2gimbal, Eigen::Matrix3d R_gimbal2world,
        Eigen::Vector3d t_camera2gimbal)
        : R_camera2gimbal_(R_camera2gimbal)
        , R_gimbal2world_(R_gimbal2world)
        , t_camera2gimbal_(t_camera2gimbal) { }

    std::shared_ptr<world_exe::interfaces::IArmorInCamera> SolvePnp(
        std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
        std::vector<data::ArmorCameraSpacing> armor_plates;

        for (int i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); i++) {
            const auto& armor_id = util::enumeration::GetArmorIdFlag(i);
            const auto& armors   = armors_in_image->GetArmors(armor_id);
            for (const auto& armor : armors) {
                auto solved_armor = Solve(armor);
                if (solved_armor.has_value()) {
                    armor_plates.emplace_back(solved_armor.value());
                }
            }
        }
        return std::make_shared<SolvedArmor>(armor_plates);
    }

    std::optional<world_exe::data::ArmorCameraSpacing> Solve(
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

        Eigen::Quaterniond orientation_in_camera(R_armor2camera);

        world_exe::data::ArmorCameraSpacing armor_in_camera;
        auto orientation_ros = util::coordinate::opencv2ros_orientation(orientation_in_camera);
        orientation_ros.normalize();
        armor_in_camera.id          = armor_in_image.id;
        armor_in_camera.position    = util::coordinate::opencv2ros_position(xyz_in_camera);
        armor_in_camera.orientation = orientation_ros;

        if (armor_in_camera.position.norm() > MaxArmorDistance) {
            return {};
        }
        return armor_in_camera;
    }

    world_exe::data::ArmorGimbalControlSpacing OptimizeYaw(
        const world_exe::data::ArmorImageSpacing& armor_in_image,
        const world_exe::data::ArmorCameraSpacing& armor_in_camera) const {

        Eigen::Vector3d armor_in_camera_ocv =
            util::coordinate::ros2opencv_position(armor_in_camera.position);
        Eigen::Vector3d armor_xyz_in_gimbal =
            R_camera2gimbal_ * armor_in_camera_ocv + t_camera2gimbal_;
        Eigen::Vector3d armor_xyz_in_world = R_gimbal2world_ * armor_xyz_in_gimbal; // why no t?

        Eigen::Quaterniond armor_orientation_in_camera_ocv =
            util::coordinate::ros2opencv_orientation(armor_in_camera.orientation);
        Eigen::Matrix3d R_armor2camera_ocv    = armor_orientation_in_camera_ocv.toRotationMatrix();
        Eigen::Matrix3d R_armor2gimbal        = R_camera2gimbal_ * R_armor2camera_ocv;
        Eigen::Matrix3d R_armor2world_initial = R_gimbal2world_ * R_armor2gimbal;

        Eigen::Vector3d armor_ypr_in_world =
            util::math::matrix_to_euler(R_armor2world_initial, 2, 1, 0);
        // double initial_yaw = armor_ypr_in_world[0];

        Eigen::Vector3d gimbal_ypr = util::math::matrix_to_euler(R_gimbal2world_, 2, 1, 0);

        constexpr double SEARCH_RANGE = 140; // degree
        auto yaw0 = util::math::clamp_pm_pi(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

        auto min_error = 1e10;
        auto best_yaw  = armor_ypr_in_world[0];

        for (int i = 0; i < SEARCH_RANGE; i++) {
            double yaw = util::math::clamp_pm_pi(yaw0 + i * CV_PI / 180.0);
            auto error = ArmorReprojectionError(
                armor_in_image, armor_xyz_in_world, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

            if (error < min_error) {
                min_error = error;
                best_yaw  = yaw;
            }
        }

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

    const std::time_t& GetTimeStamp() const { return time_stamp_; }

private:
    std::vector<cv::Point2d> ReprojectArmor(const Eigen::Vector3d& xyz_in_world, const double& yaw,
        const bool& is_large, const enumeration::ArmorIdFlag& id) const {
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
            R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
        Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose()
            * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

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

    double ArmorReprojectionError(const world_exe::data::ArmorImageSpacing& armor_in_image,
        const Eigen::Vector3d& armor_xyz_in_world, const double& yaw,
        const double& inclined) const {

        auto image_points =
            ReprojectArmor(armor_xyz_in_world, yaw, armor_in_image.isLargeArmor, armor_in_image.id);
        auto error = 0.0;
        for (int i = 0; i < 4; i++)
            error += cv::norm(armor_in_image.image_points[i] - image_points[i]);
        // auto error = SJTU_cost(image_points, armor_in_image.image_points, inclined);

        return error;
    }

    double OupostReprojectionError(world_exe::data::ArmorImageSpacing armor_in_image,
        Eigen::Vector3d armor_xyz_in_world, const double& pitch) const {

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
        Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
        Eigen::Matrix3d R_armor2world  = R_gimbal2world_ * R_armor2gimbal;
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
            R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
        Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose()
            * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

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

private:
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;
    Eigen::Vector3d t_camera2gimbal_;
    inline constexpr static const double MaxArmorDistance = 15.0;

    std::time_t time_stamp_ { 0 };
};

Solver::Solver(Eigen::Matrix3d R_camera2gimbal, Eigen::Matrix3d R_gimbal2world,
    Eigen::Vector3d t_camera2gimbal)
    : pimpl_(std::make_unique<SolverImpl>(R_camera2gimbal, R_gimbal2world, t_camera2gimbal)) { }
Solver::~Solver() = default;

const std::time_t& Solver::GetTimeStamp() const { return pimpl_->GetTimeStamp(); }

std::shared_ptr<world_exe::interfaces::IArmorInCamera> Solver::SolvePnp(
    std::shared_ptr<interfaces::IArmorInImage> armors_in_image) {
    return pimpl_->SolvePnp(armors_in_image);
}

}