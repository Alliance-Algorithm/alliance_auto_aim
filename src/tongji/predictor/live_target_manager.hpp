#pragma once

#include <memory>

#include "enum/armor_id.hpp"
#include "interfaces/target_predictor.hpp"


namespace world_exe::tongji::predictor {

class LiveTargetManager final : public interfaces::ITargetPredictor {
public:
    LiveTargetManager();
    ~LiveTargetManager();

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) override;

    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}