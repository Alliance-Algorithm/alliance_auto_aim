#include "tongji/utils/coordinate.hpp"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace coord = world_exe::util::coordinate;

TEST(CoordinateTest, Opencv2RosPosition_SimpleAxis) {
    // 1. 准备 (Arrange) - 在 OpenCV 坐标系下，沿着每个轴单位移动
    Eigen::Vector3d opencv_pos_x(1.0, 0.0, 0.0); // 沿 X 轴 (右)
    Eigen::Vector3d opencv_pos_y(0.0, 1.0, 0.0); // 沿 Y 轴 (下)
    Eigen::Vector3d opencv_pos_z(0.0, 0.0, 1.0); // 沿 Z 轴 (前)

    // 预期结果 (ROS 坐标系: x->前, y->左, z->上)
    // (x, y, z) -> (z, -x, -y)
    Eigen::Vector3d ros_expected_x(0.0, -1.0, 0.0); // (0, -1, 0)
    Eigen::Vector3d ros_expected_y(0.0, 0.0, -1.0); // (0, 0, -1)
    Eigen::Vector3d ros_expected_z(1.0, 0.0, 0.0);  // (1, 0, 0)

    // 2. 执行 (Act)
    Eigen::Vector3d ros_actual_x = coord::opencv2ros_position(opencv_pos_x);
    Eigen::Vector3d ros_actual_y = coord::opencv2ros_position(opencv_pos_y);
    Eigen::Vector3d ros_actual_z = coord::opencv2ros_position(opencv_pos_z);

    // 3. 断言 (Assert)
    // 使用 ASSERT_TRUE 结合 Eigen 的 isApprox() 方法进行浮点数向量的精确比较
    ASSERT_TRUE(ros_actual_x.isApprox(ros_expected_x)) << "X-axis conversion failed.";
    ASSERT_TRUE(ros_actual_y.isApprox(ros_expected_y)) << "Y-axis conversion failed.";
    ASSERT_TRUE(ros_actual_z.isApprox(ros_expected_z)) << "Z-axis conversion failed.";
}

TEST(CoordinateTest, Ros2OpencvPosition_SimpleAxis) {
    // 1. 准备 (Arrange) - ROS 坐标系下的单位向量
    Eigen::Vector3d ros_pos_x(1.0, 0.0, 0.0); // 沿 X 轴 (前)
    Eigen::Vector3d ros_pos_y(0.0, 1.0, 0.0); // 沿 Y 轴 (左)
    Eigen::Vector3d ros_pos_z(0.0, 0.0, 1.0); // 沿 Z 轴 (上)

    // 预期结果 (OpenCV 坐标系: x->右, y->下, z->前)
    // (-y, -z, x)
    Eigen::Vector3d opencv_expected_x(0.0, 0.0, 1.0);  // (0, 0, 1)
    Eigen::Vector3d opencv_expected_y(-1.0, 0.0, 0.0); // (-1, 0, 0)
    Eigen::Vector3d opencv_expected_z(0.0, -1.0, 0.0); // (0, -1, 0)

    // 2. 执行 (Act)
    Eigen::Vector3d opencv_actual_x = coord::ros2opencv_position(ros_pos_x);
    Eigen::Vector3d opencv_actual_y = coord::ros2opencv_position(ros_pos_y);
    Eigen::Vector3d opencv_actual_z = coord::ros2opencv_position(ros_pos_z);

    // 3. 断言 (Assert)
    ASSERT_TRUE(opencv_actual_x.isApprox(opencv_expected_x));
    ASSERT_TRUE(opencv_actual_y.isApprox(opencv_expected_y));
    ASSERT_TRUE(opencv_actual_z.isApprox(opencv_expected_z));
}

// **关键交叉测试 (Round Trip Test):** 确保来回转换后结果不变
TEST(CoordinateTest, Position_RoundTrip) {
    // 1. 准备：一个复杂的随机向量
    Eigen::Vector3d original_pos(1.23, -4.56, 7.89);

    // 2. 执行：OpenCV -> ROS -> OpenCV
    Eigen::Vector3d ros_pos   = coord::opencv2ros_position(original_pos);
    Eigen::Vector3d final_pos = coord::ros2opencv_position(ros_pos);

    // 3. 断言：最终位置应该近似等于原始位置
    ASSERT_TRUE(final_pos.isApprox(original_pos))
        << "Round trip conversion failed. Original: " << original_pos.transpose()
        << ", Final: " << final_pos.transpose();
}

TEST(CoordinateTest, Opencv2RosRotation_Identity) {
    // 1. 准备：单位旋转矩阵 (无旋转)
    Eigen::Matrix3d identity_rot = Eigen::Matrix3d::Identity();

    // 2. 执行
    Eigen::Matrix3d ros_rot = coord::opencv2ros_rotation(identity_rot);

    // 3. 断言：转换一个单位矩阵，结果应该是一个单位矩阵
    ASSERT_TRUE(ros_rot.isApprox(identity_rot)) << "Identity rotation failed.";
}

TEST(CoordinateTest, Opencv2RosRotation_KnownRotation) {
    // 1. 准备：绕 OpenCV 坐标系 Z 轴旋转 90 度 (Z-axis 绕前方向)
    // OpenCV 坐标系：X(右), Y(下), Z(前)
    // 绕 Z 轴旋转 90 度 (右转 90) => X->Y, Y->-X
    double angle = M_PI / 2.0;
    Eigen::Matrix3d rot_opencv;
    rot_opencv << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;

    // 2. 执行
    Eigen::Matrix3d ros_rot = coord::opencv2ros_rotation(rot_opencv);

    // 3. 预期结果：验证一个点。
    // 在 OpenCV 中 (1, 0, 0) 旋转后变成 (0, 1, 0)。
    // 转换到 ROS 坐标系后:
    // (1, 0, 0) -> (0, -1, 0) (ROS X)
    // (0, 1, 0) -> (0, 0, -1) (ROS Y)
    // ROS 旋转矩阵应该把 (0, -1, 0) 变成 (0, 0, -1)。

    Eigen::Vector3d ros_vector_in(0.0, -1.0, 0.0); // 相当于 OpenCV 的 X 轴
    Eigen::Vector3d ros_vector_out = ros_rot * ros_vector_in;

    Eigen::Vector3d ros_expected_out(0.0, 0.0, -1.0); // 相当于 OpenCV 的 Y 轴

    // 4. 断言
    ASSERT_TRUE(ros_vector_out.isApprox(ros_expected_out)) << "Known rotation test failed.";
}

TEST(CoordinateTest, Rotation_RoundTrip) {
    // 1. 准备：一个绕随机轴旋转的复杂旋转矩阵
    Eigen::AngleAxisd aa(0.5, Eigen::Vector3d(0.1, 0.5, -0.3).normalized());
    Eigen::Matrix3d original_rot = aa.toRotationMatrix();

    // 2. 执行：OpenCV -> ROS -> OpenCV
    Eigen::Matrix3d ros_rot   = coord::opencv2ros_rotation(original_rot);
    Eigen::Matrix3d final_rot = coord::ros2opencv_rotation(ros_rot); // 使用 ros2opencv_rotation

    // 3. 断言：最终矩阵应该近似等于原始矩阵
    ASSERT_TRUE(final_rot.isApprox(original_rot, 1e-6)) << "Rotation Round trip conversion failed.";
}
