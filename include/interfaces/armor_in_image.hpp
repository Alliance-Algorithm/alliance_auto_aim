#pragma once

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "time_stamped.hpp"
#include <opencv2/core/core.hpp>
#include <vector>

namespace world_exe::interfaces {
class IArmorInImage {

public:
    COMBINE_TIME_STAMPED;

    virtual const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const = 0;
};
}