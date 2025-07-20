#pragma once

#include "armor_in_gimbal_control.hpp"
#include "enum/armor_id.hpp"
#include <ctime>

namespace world_exe::interfaces {
class ITargetPredictor {
    virtual const IArmorInGimbalControl& Predictor(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp);
};
}