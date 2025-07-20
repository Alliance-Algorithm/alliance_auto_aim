#pragma once

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "time_stamped.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace world_exe::interfaces {
class IArmorInImage {

    COMBINE_TIME_STAMPED;

    virtual const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const;
};
}