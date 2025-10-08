#pragma once

#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <numeric>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"
#include "extended_kalman_filter.hpp"
#include "predict_model.hpp"
#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::predictor {

class LiveTarget {
public:
    LiveTarget(const Eigen::Vector3d& armor_xyz_in_world, const Eigen::Vector3d& armor_ypr_in_world,
        const enumeration::CarIDFlag& car_id)
        : last_see_time_stamp_(std::time(nullptr))
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

        ExtendedKalmanFilter<11, 4>::XVec x0;
        x0 << center_x, 0, center_y, 0, center_z, 0, armor_ypr_in_world[0], 0, model_.GetRadius(),
            0, 0;

        ExtendedKalmanFilter<11, 4>::PMat P0 = model_.GetP0Dig().asDiagonal();
        ekf_               = ExtendedKalmanFilter<11, 4>(
            x0, P0, model_.x_add); // 初始化滤波器（预测量、预测量协方差）
    }

    ExtendedKalmanFilter<11, 4>::XVec GetEkfX() const { return ekf_.x; }
    ExtendedKalmanFilter<11, 4>::PDig GetP0Dig() const { return model_.GetP0Dig(); }
    const PredictModel& GetModel() const { return model_; }
    predictor::TimeStamp LastSeen() const { return predictor::TimeStamp(last_see_time_stamp_); }

    std::vector<data::ArmorGimbalControlSpacing> GetArmorGimbalControlSpacings() const {
        std::vector<data::ArmorGimbalControlSpacing> armors;
        for (int id = 0; id < model_.GetArmorNum(); id++) {
            auto angle =
                util::math::clamp_pm_pi(this->ekf_.x[6] + id * 2 * CV_PI / model_.GetArmorNum());
            auto xyz = model_.h_armor_xyz(this->ekf_.x, id);

            data::ArmorGimbalControlSpacing armor;
            armor.id          = model_.GetID();
            armor.position    = xyz;
            armor.orientation = util::math::euler_to_quaternion(angle, 15. / 180. * CV_PI, 0);
            armors.emplace_back(std::move(armor));
        }
        return armors;
    }

    void Update(const double& dt, const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world) {
        // 装甲板匹配
        int id =
            model_.MatchArmor(ekf_.x, armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world);
        last_id = id;
        update_count++;

        Update_ypda(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id, dt);

        last_see_time_stamp_ = std::time(nullptr);
    }

    bool IsConverged() const { return EvaluateConvergence(); }

private:
    bool EvaluateConvergence() const {
        // 前哨站特殊判断
        const int required_count = (model_.GetID() == enumeration::CarIDFlag::Outpost) ? 10 : 3;
        if (update_count < required_count) return false;
        if (EvaluateDivergence()) return false;

        auto nis_failures =
            std::accumulate(ekf_.recent_nis_failures.begin(), ekf_.recent_nis_failures.end(), 0);
        if (nis_failures > 0.4 * ekf_.window_size) return false;

        return true;
    }

    bool EvaluateDivergence() const {
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
        auto H = model_.H(ekf_.x, id);
        auto R = model_.R(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id);
        auto A = model_.A(dt);
        auto Q = model_.Q(dt);
        auto f = model_.f;
        auto h = [this, id](const ExtendedKalmanFilter<11, 4>::XVec& x) { return model_.h(x, id); };
        auto z_subtract = model_.z_subtract;

        const Eigen::Vector3d& ypd = armor_ypd_in_world;
        const Eigen::Vector3d& ypr = armor_ypr_in_world;

        // 获得观测量
        ExtendedKalmanFilter<11, 4>::ZVec z(4);
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf_.Update(dt, A, Q, f, z, H, R, h, z_subtract);

        // 前哨站转速特判
        if (model_.GetID() == enumeration::CarIDFlag::Outpost && EvaluateConvergence()) {
            constexpr double max_outpost_w = 2.51;
            if (std::abs(ekf_.x[7]) > 2.0) {
                ekf_.x[7] = ekf_.x[7] > 0 ? max_outpost_w : -max_outpost_w;
            }
        }
    }

    std::time_t last_see_time_stamp_;
    ExtendedKalmanFilter<11, 4> ekf_;
    PredictModel model_;

    int last_id      = -1;
    int update_count = 0;
};
}