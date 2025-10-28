#pragma once

#include <memory>

#include "data/predictor_update_package.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::predictor {

class LiveTargetManager final : public interfaces::ITargetPredictor {
public:
    LiveTargetManager(const std::string& config_path, const double& timeout_sec = 0.1);
    ~LiveTargetManager();

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const data::TimeStamp& time_stamp) override;
    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

    void Update(std::shared_ptr<data::PredictorUpdatePackage> data,
        const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image);

    auto GetAllowedTargetID() const -> enumeration::ArmorIdFlag const;

    LiveTargetManager(const LiveTargetManager&)                = delete;
    LiveTargetManager& operator=(const LiveTargetManager&)     = delete;
    LiveTargetManager(LiveTargetManager&&) noexcept            = default;
    LiveTargetManager& operator=(LiveTargetManager&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
