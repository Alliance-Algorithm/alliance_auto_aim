#pragma once

#include <memory>

#include "enum/armor_id.hpp"
#include "interfaces/target_predictor.hpp"
#include "target.hpp"

namespace world_exe::tongji::predictor {

class TargetPredict final : public interfaces::ITargetPredictor {
public:
    TargetPredict(std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> targets);
    ~TargetPredict();

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) override;

    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}