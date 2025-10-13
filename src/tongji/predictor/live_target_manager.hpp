#pragma once

#include <memory>

#include "enum/armor_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::predictor {

class LiveTargetManager final : public interfaces::ITargetPredictor {
public:
    LiveTargetManager(const double& time_delay, const double& yaw_offset,
        const double& pitch_offset, double timeout_sec = 0.1);
    ~LiveTargetManager();

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) override;
    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
        const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image, const std::time_t& now,
        const double& bullet_speed);

    auto GetAllowedTargetID() const -> enumeration::ArmorIdFlag const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}