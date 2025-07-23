#pragma once

#define BACKWARD_HAS_DW 1
// #include "backward.hpp"
#include "enum/armor_id.hpp"
#include <stdexcept>

namespace world_exe::util::enumeration {
static inline int GetIndex(const world_exe::enumeration::ArmorIdFlag& flag) {
    const auto value = static_cast<uint32_t>(flag);
    if (value != 0 && (value & (value - 1)) == 0) return std::countr_zero(value);
    else {

        // auto st = backward::StackTrace();
        // st.load_here(4);
        // backward::Printer ptr;
        // ptr.object     = false;
        // ptr.color_mode = backward::ColorMode::always;
        // ptr.address    = false;
        // ptr.snippet    = false;
        // ptr.print(st, stderr);
        throw std::runtime_error("Invalid ArmorIdFlag value: " + std::to_string(value));
    }
}
}