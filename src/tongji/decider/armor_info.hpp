

#pragma once

#include "data/armor_image_spaceing.hpp"

namespace world_exe::tongji::decider {

enum class ArmorPriority {
    First = 1, //
    Second,    //
    Third,     //
    Forth,     //
    Fifth      //
};

struct ArmorInfo {
    ArmorInfo(const data::ArmorImageSpacing& armor_spacing,
        const ArmorPriority& priority = ArmorPriority::Fifth)
        : armor_spacing(armor_spacing)
        , priority(priority) { }

    data::ArmorImageSpacing armor_spacing;
    ArmorPriority priority;
};
}