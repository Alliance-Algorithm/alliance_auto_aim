#pragma once

#include "armor_in_gimbal_control.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/predictor.hpp"
#include "predictor_update_package.hpp"
#include <ctime>
#include <memory>

namespace world_exe::interfaces {
class ITargetPredictor {
public:
    virtual const IArmorInGimbalControl& Predict(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp)           = 0;
    virtual void Update(std::shared_ptr<IPreDictorUpdatePackage> data)               = 0;
    virtual const IPredictor& GetPredictor(const enumeration::ArmorIdFlag& id) const = 0;
};
}