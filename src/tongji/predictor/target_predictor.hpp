#pragma once

#include <memory>

#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::predictor {
class TargetPredictor final : public interfaces::ITargetPredictor {
public:
    TargetPredictor(const enumeration::ArmorIdFlag& id, std::time_t t);
    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) override;

    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}