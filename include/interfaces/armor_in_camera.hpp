#pragma once

#include "data/armor_camera_spacing.hpp"
#include "enum/armor_id.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace world_exe::interfaces {
class IArmorInCamera {
    virtual const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorId& armor_id);
};
}