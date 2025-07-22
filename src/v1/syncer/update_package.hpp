#pragma once

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/predictor_update_package.hpp"

namespace world_exe::v1::sync {
class PredictorUpdatePackage : public interfaces::IPreDictorUpdatePackage {
public:
    PredictorUpdatePackage();
    PredictorUpdatePackage(const PredictorUpdatePackage&);
    ~PredictorUpdatePackage();

    const interfaces::ITimeStamped& GetTimeStamped() const override;
    const interfaces::IArmorInCamera& GetArmors() const override;
    const Eigen::Affine3d& GetTransform() const override;

    void SetArmor(const interfaces::IArmorInCamera& armor_in_camera);
    void SetTransform(const Eigen::Affine3d& transform);
    void SetTimeStamp(const std::time_t& time_stamp);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}