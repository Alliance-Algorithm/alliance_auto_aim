#pragma once

#include "data/sync_data.hpp"

namespace world_exe::ros {

struct alignas(8) SyncData_Feb_TimeCameraGimbal_8byteAlignas {
    uint64_t time_stamp;
    double camera_to_gimbal_translation_x;
    double camera_to_gimbal_translation_y;
    double camera_to_gimbal_translation_z;
    double camera_to_gimbal_rotation_w;
    double camera_to_gimbal_rotation_x;
    double camera_to_gimbal_rotation_y;
    double camera_to_gimbal_rotation_z;
    double gimbal_to_muzzle_translation_x;
    double gimbal_to_muzzle_translation_y;
    double gimbal_to_muzzle_translation_z;
    double gimbal_to_muzzle_rotation_w;
    double gimbal_to_muzzle_rotation_x;
    double gimbal_to_muzzle_rotation_y;
    double gimbal_to_muzzle_rotation_z;
};
static data::CameraGimbalMuzzleSyncData sync_data_process(
    const SyncData_Feb_TimeCameraGimbal_8byteAlignas& data);
} // namespace world_exe::ros