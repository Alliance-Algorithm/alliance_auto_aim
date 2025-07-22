#pragma once

#include "interfaces/armor_in_gimbal_control.hpp"
#include "predict_time_stamp.hpp"

namespace world_exe::v1::predictor {
class PredictArmorInGimbalControl : public world_exe::interfaces::IArmorInGimbalControl {
public:
    PredictArmorInGimbalControl();
    PredictArmorInGimbalControl(
        const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
        const predictor::PredictTimeStamp& predict_time_stamp);
    ~PredictArmorInGimbalControl();

    void Set(const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
        const predictor::PredictTimeStamp& predict_time_stamp);

    void SetWithSingleId(const std::vector<data::ArmorGimbalControlSpacing>& armors,
        const predictor::PredictTimeStamp& predict_time_stamp);

    const std::vector<data::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override;

    const interfaces::ITimeStamped& GetTimeStamped() const override;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}