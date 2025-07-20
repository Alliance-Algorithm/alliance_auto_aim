#pragma once

#include <bit>
#include <cstdint>

namespace world_exe::enumeration {
enum class ArmorId : uint32_t {
    Unknow      = std::bit_cast<uint32_t>(-1),
    Hero        = 0b00000001,
    Engineer    = 0b00000010,
    InfantryIII = 0b00000100,
    InfantryIV  = 0b00001000,
    InfantryV   = 0b00010000,
    Sentry      = 0b00100000,
    Base        = 0b01000000,
    Outpost     = 0b10000000,
};
}