#pragma once

#include <cstdint>

namespace world_exe::enumeration {
enum class SystemVersion : uint32_t

{
    V1       = 0x00010000,
    V2       = 0x00020000,
    Balabala = 0x00030000,
    Default  = V1
};

}