#pragma once

#include <memory>

#include "interfaces/target_predictor.hpp"
#include "tongji/predictor/car_predictor_manager.hpp"

namespace world_exe::tongji::predictor {
class CarPredictorBoss final : public interfaces::ITargetPredictor {
public:
    CarPredictorBoss(std::shared_ptr<CarPredictorManager> manager);

    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}