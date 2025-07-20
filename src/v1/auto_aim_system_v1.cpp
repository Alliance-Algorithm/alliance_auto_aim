#include "./auto_aim_system_v1.hpp"

class world_exe::v1::SystemV1::SystemV1Impl { };

void world_exe::v1::SystemV1::Build() {
    if (SystemV1::instance_ != nullptr) return;
    instance_ = new SystemV1Impl();
}