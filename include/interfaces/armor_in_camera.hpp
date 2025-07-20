#pragma once

#include "data/armor_camera_spacing.hpp"
#include "enum/armor_id.hpp"
#include "time_stamped.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {
class IArmorInCamera {
public:
    COMBINE_TIME_STAMPED;

    virtual const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const;
};
}