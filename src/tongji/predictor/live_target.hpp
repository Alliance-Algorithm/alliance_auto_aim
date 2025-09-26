#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>
#include <ctime>
#include <vector>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"
#include "extended_kalman_filter.hpp"
#include "predict_model.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

struct TargetStatus {
    bool jumped       = false;
    bool switched     = false;
    bool converged    = false;
    bool diverged     = false;
    bool lost         = false;
    bool reidentified = false;
    int last_id       = -1;
    double lock_id_   = -1;
    int switch_count  = 0;
    int update_count  = 0;
    int lost_count    = 0;
};

class LiveTarget {
public:
    TargetStatus status_;

    LiveTarget(const Eigen::Vector3d& armor_xyz_in_world, const Eigen::Vector3d& armor_ypr_in_world,
        const enumeration::CarIDFlag& car_id, const std::time_t& t)
        : last_time_stamp_(t)
        , model_(car_id) {

        // x vx y vy z vz a w r l h
        // a: angle
        // w: angular velocity
        // l: r2 - r1
        // h: z2 - z1
        auto center_x =
            armor_xyz_in_world[0] + model_.GetRadius() * std::cos(armor_ypr_in_world[0]);
        auto center_y =
            armor_xyz_in_world[1] + model_.GetRadius() * std::sin(armor_ypr_in_world[0]);
        auto center_z = armor_xyz_in_world[2];

        Eigen::VectorXd x0(11);
        x0 << center_x, 0, center_y, 0, center_z, 0, armor_ypr_in_world[0], 0, model_.GetRadius(),
            0, 0;

        Eigen::MatrixXd P0 = model_.GetP0Dig().asDiagonal();
        ekf_ = ExtendedKalmanFilter(x0, P0, model_.x_add); // 初始化滤波器（预测量、预测量协方差）
    }

    Eigen::VectorXd GetEkfX() const { return ekf_.x; }
    Eigen::VectorXd GetP0Dig() const { return model_.GetP0Dig(); }
    const PredictModel& GetModel() const { return model_; }
    std::time_t GetTimeStamp() const { return last_time_stamp_; }

    data::ArmorGimbalControlSpacing GetTargetArmorGimbalControlSpacings() {
        const auto& ekf_x     = GetEkfX();
        const auto& xyza_list = model_.GetArmorXYZAList(ekf_x);

        Eigen::Vector4d xyza = xyza_list.at(0);
        data::ArmorGimbalControlSpacing armor;

        if (!status_.jumped) {
            // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
            xyza = xyza_list.at(0);
        }

        // 整车旋转中心的球坐标yaw
        auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
        std::vector<double> delta_angle_list;
        for (int i = 0; i < model_.GetArmorNum(); i++) {
            auto delta_angle = util::math::clamp_pm_pi(xyza_list[i][3] - center_yaw);
            delta_angle_list.emplace_back(delta_angle);
        }

        // 不考虑小陀螺
        if (std::abs(ekf_x[8]) <= 2 && model_.GetID() != enumeration::CarIDFlag::Outpost) {
            // 选择在可射击范围内的装甲板
            std::vector<int> id_list;
            for (int i = 0; i < model_.GetArmorNum(); i++) {
                if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
                id_list.push_back(i);
            }

            // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
            if (id_list.size() > 1) {
                int id0 = id_list[0], id1 = id_list[1];

                // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
                if (status_.lock_id_ != id0 && status_.lock_id_ != id1)
                    status_.lock_id_ =
                        (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0
                                                                                            : id1;
                xyza = xyza_list.at(status_.lock_id_);
            }

            // 只有一个装甲板在可射击范围内时，退出锁定模式
            status_.lock_id_ = -1;
            xyza             = xyza_list.at(id_list[0]);
        }

        double coming_angle, leaving_angle;
        if (model_.GetID() == enumeration::CarIDFlag::Outpost) {
            coming_angle  = 70 / 57.3;
            leaving_angle = 30 / 57.3;
        } else {
            coming_angle  = comming_angle_;
            leaving_angle = leaving_angle_;
        }

        // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
        for (int i = 0; i < model_.GetArmorNum(); i++) {
            if (std::abs(delta_angle_list[i]) > coming_angle) continue;
            if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) xyza = xyza_list[i];
            else if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) xyza = xyza_list[i];
        }

        armor.id          = model_.GetID();
        armor.position    = xyza.head<3>();
        const auto& angle = xyza[3];
        armor.orientation = util::math::euler_to_quaternion(angle, 15. / 180. * CV_PI, 0);

        return armor;
    }

    void Update(const double& dt, const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world) {

        // 装甲板匹配
        int id =
            model_.MatchArmor(ekf_.x, armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world);
        if (id != 0) status_.jumped = true;
        status_.switched = (id != status_.last_id);
        if (status_.switched) status_.switch_count++;

        status_.last_id = id;
        Update_ypda(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id, dt);
        status_.update_count++;
    }

private:
    bool Converged() {
        if (model_.GetID() != enumeration::CarIDFlag::Outpost && status_.update_count > 3
            && !this->Diverged()) {
            status_.converged = true;
        }

        // 前哨站特殊判断
        if (model_.GetID() == enumeration::CarIDFlag::Outpost && status_.update_count > 10
            && !this->Diverged()) {
            status_.converged = true;
        }
        return status_.converged;
    }

    bool Diverged() const {
        auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
        auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

        if (r_ok && l_ok) return false;
        // util::logger::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
        return true;
    }

    void Update_ypda(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world,
        const int& id, const double& dt) {
        // 观测jacobi
        auto H          = model_.H(ekf_.x, id);
        auto R          = model_.R(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id);
        auto A          = model_.A(dt);
        auto Q          = model_.Q(dt);
        auto f          = model_.f;
        auto h          = [this, id](const Eigen::VectorXd& x) { return model_.h(x, id); };
        auto z_subtract = model_.z_subtract;

        const Eigen::VectorXd& ypd = armor_ypd_in_world;
        const Eigen::VectorXd& ypr = armor_ypr_in_world;

        // 获得观测量
        Eigen::VectorXd z(4);
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf_.Update(dt, A, Q, f, z, H, R, h, z_subtract);

        // 前哨站转速特判
        if (this->Converged() && model_.GetID() == enumeration::CarIDFlag::Outpost
            && std::abs(this->ekf_.x[7]) > 2)
            this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;
    }

    std::time_t last_time_stamp_;
    ExtendedKalmanFilter ekf_;
    PredictModel model_;

    double comming_angle_ = 60 / 57.3;
    double leaving_angle_ = 20 / 57.3;
};
}