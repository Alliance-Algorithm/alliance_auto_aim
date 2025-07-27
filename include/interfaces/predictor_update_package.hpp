#pragma once

#include "./armor_in_camera.hpp"

#include "interfaces/time_stamped.hpp"
#include <memory>

namespace world_exe::interfaces {
class IPreDictorUpdatePackage {
public:
    COMBINE_TIME_STAMPED;
    virtual std::shared_ptr<IArmorInCamera> GetArmors() const = 0;

    ///
    /// Affine form image to gimbal_control
    virtual Eigen::Affine3d GetTransform() const = 0;
};
}