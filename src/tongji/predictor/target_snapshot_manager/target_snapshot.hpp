#pragma once

#include <ctime>

#include "../../time_stamp/time_stamp.hpp"
#include "../kalman_filter/extended_kalman_filter.hpp"
#include "../kalman_filter/predict_model.hpp"
#include "../live_target_manager/live_target.hpp"

namespace world_exe::tongji::predictor {

class TargetSnapshot {
public:
    using PredictorModel = EKFModel<11, 4>;
    using EKF            = ExtendedKalmanFilter<PredictorModel>;

    TargetSnapshot(const LiveTarget& target)
        : model_(target.GetModel())
        , ekf_(target.GetEkfX(), target.GetP0Dig().asDiagonal(), model_)
        , time_stamp_(target.LastSeen()) { }

    // std::vector<data::ArmorGimbalControlSpacing> GetArmorGimbalControlSpacings() const {
    //     std::vector<data::ArmorGimbalControlSpacing> armors;
    //     for (int id = 0; id < model_.GetArmorNum(); id++) {
    //         auto angle =
    //             util::math::clamp_pm_pi(this->ekf_.x[6] + id * 2 * CV_PI / model_.GetArmorNum());
    //         auto xyz = model_.h_armor_xyz(this->ekf_.x, id);

    //         data::ArmorGimbalControlSpacing armor;
    //         armor.id          = model_.GetID();
    //         armor.position    = xyz;
    //         armor.orientation = util::math::euler_to_quaternion(angle, 15. / 180. * CV_PI, 0);
    //         armors.emplace_back(std::move(armor));
    //     }
    //     return armors;
    // }
    // TODO：
    auto GetPredictedXYZAList(const double& dt) -> std::vector<Eigen::Vector4d> const {
        return model_.GetArmorXYZAList(this->Predict(dt));
    }

    auto GetTimeStamp() const { return time_stamp_; }
    auto GetID() const { return model_.GetID(); }
    auto GetEkfX() const { return ekf_.x; }

    auto Predict(const double& dt) -> Eigen::Vector<double, 11> const {
        auto predicted_x = model_.f(ekf_.x, dt);
        return predicted_x;
    }

private:
    PredictorModel model_;
    ExtendedKalmanFilter<PredictorModel> ekf_;
    time_stamp::TimeStamp time_stamp_;
};

}
