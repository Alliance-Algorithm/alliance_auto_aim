#pragma once

#include <memory>

#include "interfaces/predictor.hpp"
#include "tongji/predictor/target.hpp"

namespace world_exe::tongji::predictor {

class MultiTargetSnapshotPredictor final : public interfaces::IPredictor {
public:
    MultiTargetSnapshotPredictor(const std::vector<Target>& targets);
    const enumeration ::ArmorIdFlag& GetId() const override;

    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std ::time_t& time_stamp) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
