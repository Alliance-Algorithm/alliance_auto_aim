
#include "system_factory.hpp"
#include "../v1/auto_aim_system_v1.hpp"
#include "enum/system_version.hpp"
#include <format>

void world_exe::core::SystemFactory::Build(const enumeration::SystemVersion& version) {
    switch (version) {
    case enumeration::SystemVersion::V1:
        world_exe::v1::SystemV1::Build();
        break;
    default:
#if __cplusplus >= 202002L
        throw new std::runtime_error(std::format("Target version 0x{:x} is not impleme \n Factory "
                                                 "core/system_factory.cpp : Build(SystemVersion) ",
            (int)version));
#else
        throw new std::runtime_error(std::format("Target version is not impleme \n Factory "
                                                 "core/system_factory.cpp : Build(SystemVersion) "))
#endif
        break;
    }
}