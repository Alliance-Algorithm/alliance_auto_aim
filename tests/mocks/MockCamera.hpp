#include <Eigen/Dense>

#include "data/time_stamped.hpp"

namespace world_exe::tests::mock {

struct MockCamera {
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;

    MockCamera()
        : position(Eigen::Vector3d::Zero())
        , orientation(Eigen::Matrix3d::Identity()) { }
};

class MockCameraInGimbal final {
public:
    MockCameraInGimbal(double angular_speed = 1)
        : time(std::chrono::steady_clock::now().time_since_epoch())
        , camera({ })
        , gimbal_origin(Eigen::Vector3d(0, 0, 0)) { }

    const data::TimeStamp& GetTimeStamp() const { return time; }

    void RotateYawPitchAndTranslate(
        double t, double yaw_speed, double pitch_speed, const Eigen::Vector3d& linear_velocity) {
        double yaw_angle   = yaw_speed * t;
        double pitch_angle = pitch_speed * t;

        Eigen::Matrix3d R_yaw =
            Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d R_pitch =
            Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d rotation = R_yaw * R_pitch;

        Eigen::Vector3d translation = linear_velocity * t;

        MoveCamera(rotation, translation);
    }

private:
    void MoveCamera(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation) {
        Eigen::Vector3d relative = camera.position - gimbal_origin;
        camera.position          = gimbal_origin + rotation_matrix * relative + translation;
        camera.orientation       = rotation_matrix * camera.orientation;
    }

    data::TimeStamp time;
    MockCamera camera;
    Eigen::Vector3d gimbal_origin;
};
}