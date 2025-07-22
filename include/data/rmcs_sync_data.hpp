#pragma once

#include <Eigen/Eigen>
#include <ctime>
namespace world_exe::data {
struct SyncData {
    std::time_t camera_capture_begin_time_stamp;
    Eigen::Affine3d camera_to_gimbal;
    Eigen::Affine3d gimbal_to_muzzle;
};
}