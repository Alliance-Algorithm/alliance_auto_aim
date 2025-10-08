#pragma once

#include <memory>

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "interfaces/target_predictor.hpp"
#include "tongji/state_machine/state_machine.hpp"

namespace world_exe::tongji::predictor {

class LiveTargetManager final : public interfaces::ITargetPredictor {
public:
    LiveTargetManager(
        std::shared_ptr<state_machine::StateMachine> state_machine, double timeout_sec = 0.1);
    ~LiveTargetManager();

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) override;

    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

    auto GetLiveTargetIDs() -> enumeration::CarIDFlag const;

    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data, double dt);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}