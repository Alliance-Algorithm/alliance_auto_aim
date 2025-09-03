#pragma once

#include <string>

#include "enum/armor_id.hpp"

namespace world_exe::util::stringifier {
static inline std::string ToString(const world_exe::enumeration::ArmorIdFlag& id) {

    switch (id) {
    case enumeration::ArmorIdFlag::Hero:
        return "Hero";
    case enumeration::ArmorIdFlag::Engineer:
        return "Engineer";
    case enumeration::ArmorIdFlag::InfantryIII:
        return "InfantryIII";
    case enumeration::ArmorIdFlag::InfantryIV:
        return "InfantryIV";
    case enumeration::ArmorIdFlag::InfantryV:
        return "InfantryV";
    case enumeration::ArmorIdFlag::Sentry:
        return "Sentry";
    case enumeration::ArmorIdFlag::Base:
        return "Base";
    case enumeration::ArmorIdFlag::Outpost:
        return "Outpost";
    case enumeration::ArmorIdFlag::Unknow:
        return "Unknow";
    case enumeration::ArmorIdFlag::None:
        return "None";
    default:
        return "UnknownID";
    }
}
}