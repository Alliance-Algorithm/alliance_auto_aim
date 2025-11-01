#pragma once

#include <Eigen/Eigen>

namespace world_exe::util::math {

// [−π,π] 注意边界值
static constexpr double clamp_pm_pi(auto angle) {
    return std::remainder(angle, 2.0 * std::numbers::pi);
}
// zyx order
static inline Eigen::Vector3d get_ypr_from_quaternion(const Eigen::Quaterniond& quaternion) {
    const Eigen::Matrix3d R = quaternion.toRotationMatrix();
    return R.eulerAngles(2, 1, 0);
}

/**
 * @brief 将笛卡尔坐标点 (x, y) 绕原点旋转。
 * delta_angle > 0 时，执行逆时针旋转。
 * @param x 点的 x 坐标。
 * @param y 点的 y 坐标。
 * @param delta_angle 旋转角度 (弧度)。
 * @return std::pair<double, double> 旋转后的新坐标 (x', y')。
 */
static inline std::pair<double, double> rotate_point_ccw(double x, double y, double delta_angle) {
    const double cos_d = std::cos(delta_angle);
    const double sin_d = std::sin(delta_angle);

    const double x_new = x * cos_d - y * sin_d; // x' = x*cos(d) - y*sin(d)
    const double y_new = x * sin_d + y * cos_d; // y' = x*sin(d) + y*cos(d)

    return { x_new, y_new };
}

// zyx order
static Eigen::Matrix3d euler_to_matrix(const Eigen::Vector3d& ypr) {
    Eigen::AngleAxisd rollAngle(ypr[2], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ypr[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(ypr[0], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q.toRotationMatrix();
}

static inline Eigen::Quaterniond euler_to_quaternion(
    double yaw_rad, double pitch_rad, double roll_rad) {
    return Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
}

static inline Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& ypr) {
    return Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
}
// zyx order
static Eigen::Vector3d quaternion_to_euler(Eigen::Quaterniond q) {
    const Eigen::Matrix3d R = q.toRotationMatrix();
    return R.eulerAngles(2, 1, 0);
}

static Eigen::Vector3d matrix_to_euler(Eigen::Matrix3d R) { return R.eulerAngles(2, 1, 0); }

static inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    const double x = xyz[0];
    const double y = xyz[1];
    const double z = xyz[2];

    const double r_xy = std::hypot(x, y);

    const double distance = std::hypot(r_xy, z);
    const double yaw      = std::atan2(y, x);
    const double pitch    = std::atan2(z, r_xy);

    return { yaw, pitch, distance };
}

static Eigen::Matrix<double, 3, 3> xyz2ypd_jacobian(const Eigen::Vector3d& xyz) {
    const auto x = xyz[0], y = xyz[1], z = xyz[2];

    const double r_xy_sq = x * x + y * y;      // r_xy^2
    const double r_xy    = std::sqrt(r_xy_sq); // r_xy
    const double D_sq    = r_xy_sq + z * z;    // D^2 (总距离平方)
    const double D       = std::sqrt(D_sq);    // D (总距离)

    if (r_xy_sq < 1e-12) {
        return Eigen::Matrix3d::Zero();
    }

    // I. Yaw (行 0): J[0, *]
    const double r_xy_sq_inv = 1.0 / r_xy_sq;
    const double dyaw_dx     = -y * r_xy_sq_inv;
    const double dyaw_dy     = x * r_xy_sq_inv;
    const double dyaw_dz     = 0.0;

    // II. Pitch (行 1): J[1, *]
    // 简化后的公式使用 D_sq 和 r_xy
    const double D_sq_inv = 1.0 / D_sq;
    const double r_xy_inv = 1.0 / r_xy;
    const double factor   = z * D_sq_inv * r_xy_inv; // z / (D^2 * r_xy)

    const double dpitch_dx = -x * factor;
    const double dpitch_dy = -y * factor;
    // dpitch_dz = r_xy / D^2
    const double dpitch_dz = r_xy * D_sq_inv;

    // III. Distance (行 2): J[2, *]
    // dD/dx = x/D
    const double D_inv        = 1.0 / D;
    const double ddistance_dx = x * D_inv;
    const double ddistance_dy = y * D_inv;
    const double ddistance_dz = z * D_inv;

    Eigen::Matrix3d J;
    // clang-format off
    J << dyaw_dx     , dyaw_dy     , dyaw_dz     ,
         dpitch_dx   , dpitch_dy   , dpitch_dz   ,
         ddistance_dx, ddistance_dy, ddistance_dz;
    // clang-format on

    return J;
}

} // namespace rmcs_auto_aim::util::math
