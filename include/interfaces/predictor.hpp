#pragma once

#include "armor_in_gimbal_control.hpp"
#include "enum/armor_id.hpp"
#include <ctime>

namespace world_exe::interfaces {
class IPredictor {
public:
    virtual const enumeration::ArmorIdFlag& GetId() const                         = 0;
    virtual const IArmorInGimbalControl& Predictor(const std::time_t& time_stamp) = 0;
};
}