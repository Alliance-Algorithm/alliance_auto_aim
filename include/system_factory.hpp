#pragma once

#include "enum/system_version.hpp"

namespace world_exe::core {

class SystemFactory {
    static void Build(const enumeration::SystemVersion& version);
};
}