
#include "util/math_tongji.hpp"

#include <cmath>
#include <numbers>

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace math = world_exe::util::math;

constexpr double TOLERANCE = 1e-6;

// --- 1. 角度钳位测试 (clamp_pm_pi) ---
// 目标：确保将角度标准化到 [−π, π] 范围内。std::remainder(angle, 2*PI) 保证了结果在 [-PI, PI]

TEST(MathUtilsTest, ClampPi_NormalRange) {
    // 1. 正常范围内的角度
    EXPECT_NEAR(math::clamp_pm_pi(M_PI / 2.0), M_PI / 2.0, TOLERANCE);
    EXPECT_NEAR(math::clamp_pm_pi(-M_PI / 2.0), -M_PI / 2.0, TOLERANCE);
}

TEST(MathUtilsTest, ClampPi_BoundaryValues) {
    // 容忍度常量
    constexpr double TOLERANCE = 1e-6;

    // --- 1. 测试 PI + eps -> -PI + eps 的行为 ---
    // 确保大于 PI 的值被正确拉回负半轴。
    EXPECT_NEAR(math::clamp_pm_pi(std::numbers::pi + 1e-7), -std::numbers::pi + 1e-7, TOLERANCE)
        << "Failed for PI + epsilon. Expected: -PI + epsilon.";

    // --- 2. 奇数倍 PI 测试 (绝对值断言) ---
    // 目标：确保结果的绝对值是 PI，因为 std::remainder(x, 2PI) 不保证符号。

    // 测试 3*PI: 结果必须是 PI 或 -PI
    double result_3pi = math::clamp_pm_pi(3.0 * std::numbers::pi);
    EXPECT_NEAR(std::abs(result_3pi), std::numbers::pi, TOLERANCE)
        << "Failed for 3*PI. Expected absolute value PI, got: " << result_3pi;

    // 测试 -3*PI: 结果必须是 PI 或 -PI
    double result_neg_3pi = math::clamp_pm_pi(-3.0 * std::numbers::pi);
    EXPECT_NEAR(std::abs(result_neg_3pi), std::numbers::pi, TOLERANCE)
        << "Failed for -3*PI. Expected absolute value PI, got: " << result_neg_3pi;

    // --- 3. 严格的 PI 边界 ---
    // 测试 PI 本身。由于 std::remainder(PI, 2PI) 可能为 PI 或 -PI，我们只断言绝对值。
    double result_pi = math::clamp_pm_pi(std::numbers::pi);
    EXPECT_NEAR(std::abs(result_pi), std::numbers::pi, TOLERANCE)
        << "Failed for PI. Expected absolute value PI, got: " << result_pi;
}

// --- 2. 坐标旋转测试 (rotate_point_ccw) ---
// 目标：确保点 (x, y) 绕原点正确地执行逆时针 (CCW) 旋转。

TEST(MathTransformTest, RotatePointCCW_ZeroRotation) {
    // 1. 零旋转
    auto [x_new, y_new] = math::rotate_point_ccw(10.0, 5.0, 0.0);
    EXPECT_NEAR(x_new, 10.0, TOLERANCE);
    EXPECT_NEAR(y_new, 5.0, TOLERANCE);
}

TEST(MathTransformTest, RotatePointCCW_90Degrees) {
    // 2. 逆时针旋转 90 度 (PI/2)： (1, 0) -> (0, 1)
    auto [x1, y1] = math::rotate_point_ccw(1.0, 0.0, M_PI / 2.0);
    EXPECT_NEAR(x1, 0.0, TOLERANCE);
    EXPECT_NEAR(y1, 1.0, TOLERANCE);

    // (0, 1) -> (-1, 0)
    auto [x2, y2] = math::rotate_point_ccw(0.0, 1.0, M_PI / 2.0);
    EXPECT_NEAR(x2, -1.0, TOLERANCE);
    EXPECT_NEAR(y2, 0.0, TOLERANCE);
}

TEST(MathTransformTest, RotatePointCCW_RoundTrip) {
    // 3. 来回旋转：旋转 delta，再旋转 -delta，应回到原点
    double x = 3.0, y = 4.0;
    double delta_angle = M_PI / 6.0; // 30度

    auto [x_rot, y_rot]     = math::rotate_point_ccw(x, y, delta_angle);
    auto [x_final, y_final] = math::rotate_point_ccw(x_rot, y_rot, -delta_angle);

    EXPECT_NEAR(x_final, x, TOLERANCE);
    EXPECT_NEAR(y_final, y, TOLERANCE);
}

// --- 3. 欧拉角/四元数/矩阵互转 (Round Trip Tests) ---
// 目标：确保所有 ZYX 顺序转换函数是互逆的。

TEST(MathConversionTest, EulerToQuaternionAndBack_RoundTrip) {
    // 1. Euler -> Quaternion -> Euler (使用统一的 ZYX 顺序)
    Eigen::Vector3d original_ypr(0.5, -0.4, 0.3); // YAW, PITCH, ROLL

    Eigen::Quaterniond q      = math::euler_to_quaternion(original_ypr);
    Eigen::Vector3d final_ypr = math::quaternion_to_euler(q);

    // 两个函数都使用了 Eigen 的 ZYX 接口，结果应该匹配
    ASSERT_TRUE(final_ypr.isApprox(original_ypr, TOLERANCE))
        << "E->Q->E failed."
        << "\nOriginal YPR: " << original_ypr.transpose()
        << "\nFinal YPR: " << final_ypr.transpose();
}

TEST(MathConversionTest, EulerToMatrixAndBack_RoundTrip) {
    // 2. Euler -> Matrix -> Euler
    Eigen::Vector3d original_ypr(0.8, -0.1, 0.4);

    Eigen::Matrix3d R         = math::euler_to_matrix(original_ypr);
    Eigen::Vector3d final_ypr = math::matrix_to_euler(R);

    ASSERT_TRUE(final_ypr.isApprox(original_ypr, TOLERANCE))
        << "E->M->E failed."
        << "\nOriginal YPR: " << original_ypr.transpose()
        << "\nFinal YPR: " << final_ypr.transpose();
}

TEST(MathConversionTest, QuaternionToEuler_DirectAccess) {
    // 3. get_ypr_from_quaternion 测试
    Eigen::Vector3d expected_ypr(1.0, 0.5, -0.3);
    Eigen::Quaterniond q       = math::euler_to_quaternion(expected_ypr);
    Eigen::Vector3d actual_ypr = math::get_ypr_from_quaternion(q);

    ASSERT_TRUE(actual_ypr.isApprox(expected_ypr, TOLERANCE)) << "get_ypr_from_quaternion failed.";
}

// --- 4. 坐标转换测试 (xyz2ypd, xyz2ypd_jacobian) ---

TEST(MathConversionTest, XYZ2YPD_KnownPoint) {
    // 1. 沿 X 轴 (前方)
    Eigen::Vector3d xyz(10.0, 0.0, 0.0);
    Eigen::Vector3d ypd = math::xyz2ypd(xyz); // 预期 Yaw=0, Pitch=0, Dist=10

    Eigen::Vector3d expected_ypd(0.0, 0.0, 10.0);

    ASSERT_TRUE(ypd.isApprox(expected_ypd, TOLERANCE))
        << "Expected YPD: " << expected_ypd.transpose() << "\nActual YPD: " << ypd.transpose();
}

TEST(MathConversionTest, XYZ2YPD_Jacobian_KnownPoint) {
    // 2. 雅可比矩阵测试 - 检查一个已知点 (1, 0, 0)
    Eigen::Vector3d xyz(1.0, 0.0, 0.0);
    Eigen::Matrix3d J = math::xyz2ypd_jacobian(xyz);

    // 预期结果 (x=1, y=0, z=0)
    // J[0, *] (Yaw): dyaw/dx=0, dyaw/dy=1, dyaw/dz=0
    // J[1, *] (Pitch): dpitch/dx=0, dpitch/dy=0, dpitch/dz=1
    // J[2, *] (Dist): dD/dx=1, dD/dy=0, dD/dz=0
    Eigen::Matrix3d expected_J;
    expected_J << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0;

    ASSERT_TRUE(J.isApprox(expected_J, TOLERANCE)) << "Jacobian for (1,0,0) failed."
                                                   << "\nExpected:\n"
                                                   << expected_J << "\nActual:\n"
                                                   << J;
}

TEST(MathConversionTest, Jacobian_Singularity) {
    // 3. 奇点测试：(0, 0, z) 处，r_xy_sq < 1e-12 应该返回零矩阵
    Eigen::Vector3d xyz_singularity(0.0, 0.0, 5.0);
    Eigen::Matrix3d J = math::xyz2ypd_jacobian(xyz_singularity);

    ASSERT_TRUE(J.isApprox(Eigen::Matrix3d::Zero(), TOLERANCE)) << "Jacobian failed at singularity "
                                                                   "(0, 0, z)."
                                                                << "\nActual:\n"
                                                                << J;
}
