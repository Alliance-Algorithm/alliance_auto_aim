#pragma once

#include <cstdlib>
#include <ctime>
#include <optional>

#include <Eigen/Dense>

#include "../../time_stamp/time_stamp.hpp"
#include "../kalman_filter/extended_kalman_filter.hpp"
#include "../kalman_filter/predict_model.hpp"
#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"

namespace world_exe::tongji::predictor {

class LiveTarget {
public:
    using PredictorModel = EKFModel<11, 4>;
    using EKF            = ExtendedKalmanFilter<PredictorModel>;

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

        EKF::XVec x0 { center_x, 0, center_y, 0, center_z, 0, armor_ypr_in_world[0], 0,
            model_.GetRadius(), 0, 0 };

        EKF::PMat P0 = model_.GetP0Dig().asDiagonal();
        ekf_.emplace(x0, P0, model_); // 初始化滤波器（预测量、预测量协方差）
    }

    EKF::XVec GetEkfX() const { return ekf_->x; }
    EKF::PDig GetP0Dig() const { return model_.GetP0Dig(); }
    const PredictorModel& GetModel() const { return model_; }
    time_stamp::TimeStamp LastSeen() const { return time_stamp::TimeStamp(last_see_time_stamp_); }

    std::vector<data::ArmorGimbalControlSpacing> GetArmorGimbalControlSpacings() const {
        std::vector<data::ArmorGimbalControlSpacing> armors;
        for (int id = 0; id < model_.GetArmorNum(); id++) {
            auto angle =
                util::math::clamp_pm_pi(this->ekf_->x[6] + id * 2 * CV_PI / model_.GetArmorNum());
            auto xyz = model_.h_armor_xyz(this->ekf_->x, id);

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
            model_.MatchArmor(ekf_->x, armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world);
        last_id_ = id;
        update_count_++;

        Update_ypda(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id, dt);

        last_see_time_stamp_ = std::time(nullptr);
    }

    bool IsConverged() const {
        if (!ekf_.has_value()) return false;
        // 前哨站特殊判断
        const int required_count = (model_.GetID() == enumeration::CarIDFlag::Outpost) ? 10 : 3;
        if (update_count_ < required_count || IsDivergened()) return false;
        return true;
    }

private:
    bool IsDivergened() const {
        auto r_ok = ekf_->x[8] > 0.05 && ekf_->x[8] < 0.5;
        auto l_ok = ekf_->x[8] + ekf_->x[9] > 0.05 && ekf_->x[8] + ekf_->x[9] < 0.5;

        if (r_ok && l_ok) return false;
        // util::logger::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_->x[8], ekf_->x[9]);
        return true;
    }
    // TODO:need to update correctly
    void Update_ypda(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world,
        const int& id, const double& dt) {
        // 观测jacobi
        auto H = model_.H(ekf_->x, id);
        auto R = model_.R(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id);

        const Eigen::Vector3d& ypd = armor_ypd_in_world;
        const Eigen::Vector3d& ypr = armor_ypr_in_world;

        // 获得观测量
        EKF::ZVec z(4);
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf_->Update(dt, z, H, R, id);

        // 前哨站转速特判
        if (model_.GetID() == enumeration::CarIDFlag::Outpost) {
            constexpr double max_outpost_w = 2.51;
            if (std::abs(ekf_->x[7]) > 2.0) {
                ekf_->x[7] = ekf_->x[7] > 0 ? max_outpost_w : -max_outpost_w;
            }
        }
    }

    std::time_t last_see_time_stamp_;
    PredictorModel model_;
    std::optional<EKF> ekf_;

    int last_id_                           = -1;
    int update_count_                      = 0;
    const double max_allowed_failure_rate_ = 0.4;
};
}
