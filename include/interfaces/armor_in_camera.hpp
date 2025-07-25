#pragma once

#include "data/armor_camera_spacing.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/time_stamped.hpp"
#include <opencv2/core/mat.hpp>

namespace world_exe::interfaces {
class IArmorInCamera {
public:
    COMBINE_TIME_STAMPED;

    virtual const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;
};
}