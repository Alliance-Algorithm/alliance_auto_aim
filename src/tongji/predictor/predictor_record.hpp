#pragma once

#include <memory>
#include <unordered_map>

#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"
#include "tongji/predictor/target.hpp"

namespace world_exe::tongji::predictor {

class PredictorRecord final : public interfaces::IPredictor {
public:
    PredictorRecord(const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>>& snapshots);
    ~PredictorRecord();

    const enumeration ::ArmorIdFlag& GetId() const override;

    std ::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std ::time_t& time_stamp) const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
