#pragma once

#include "tongji/predictor/live_target.hpp"
#include "tongji/predictor/predict_model.hpp"
#include "util/extended_kalman_filter.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshot {
public:
    TargetSnapshot(const LiveTarget& target)
        : model_(target.GetModel())
        , ekf_(target.GetEkfX(), target.GetP0Dig().asDiagonal(), model_.x_add) { }

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

private:
    PredictModel model_;
    util::ExtendedKalmanFilter ekf_;
};

}