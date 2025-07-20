#pragma once

#include "armor_in_gimbal_control.hpp"
#include "enum/armor_id.hpp"
#include "predictor_update_package.hpp"
#include <ctime>

namespace world_exe::interfaces {
class ITargetPredictor {
public:
    virtual const IArmorInGimbalControl& Predictor(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp);
    virtual const IArmorInGimbalControl& Update(const IPreDictorUpdatePackage& data);
};
}