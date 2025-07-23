#include "sync_data_processor.hpp"
#include "Eigen/src/Geometry/Quaternion.h"
#include "Eigen/src/Geometry/Transform.h"
#include "Eigen/src/Geometry/Translation.h"
#include "data/sync_data.hpp"
#include <bit>
#include <ctime>

static world_exe::data::CameraGimbalMuzzleSyncData sync_data_process(
    const world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas &data) {
  auto gimbal_to_muzzle_affine = Eigen::Affine3d::Identity();
  gimbal_to_muzzle_affine.translate(Eigen::Translation3d{
      data.gimbal_to_muzzle_translation_x, data.gimbal_to_muzzle_translation_y,
      data.gimbal_to_muzzle_translation_z}
                                        .translation());
  gimbal_to_muzzle_affine.rotate(Eigen::Quaterniond{
      data.gimbal_to_muzzle_rotation_w, data.gimbal_to_muzzle_rotation_x,
      data.gimbal_to_muzzle_rotation_y, data.gimbal_to_muzzle_rotation_z});
  auto camera_to_gimbal_affine = Eigen::Affine3d::Identity();
  camera_to_gimbal_affine.translate(Eigen::Translation3d{
      data.camera_to_gimbal_translation_x, data.camera_to_gimbal_translation_y,
      data.camera_to_gimbal_translation_z}
                                        .translation());
  camera_to_gimbal_affine.rotate(Eigen::Quaterniond{
      data.camera_to_gimbal_rotation_w, data.camera_to_gimbal_rotation_x,
      data.camera_to_gimbal_rotation_y, data.camera_to_gimbal_rotation_z});
  return {.camera_capture_begin_time_stamp =
              std::bit_cast<time_t>(data.time_stamp),
          .camera_to_gimbal = camera_to_gimbal_affine,
          .gimbal_to_muzzle = gimbal_to_muzzle_affine};
}