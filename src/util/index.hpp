#pragma once

#include "enum/armor_id.hpp"
#include <stdexcept>

namespace world_exe::util::enumeration {
static inline int GetIndex(const world_exe::enumeration::ArmorIdFlag& flag) {
    const auto value = static_cast<uint32_t>(flag);
    if (value != 0 && (value & (value - 1)) == 0) return std::countr_zero(value) / 4;
    else throw std::runtime_error("Invalid ArmorIdFlag value: " + std::to_string(value)); }
}