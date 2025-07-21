#pragma once

#include "interfaces/armor_in_gimbal_control.hpp"
#include "predict_time_stamp.hpp"

namespace world_exe::predictor {
class PredictArmorInGimbalControl : public world_exe::interfaces::IArmorInGimbalControl {
public:
    PredictArmorInGimbalControl() = default;
    PredictArmorInGimbalControl(const std::vector<data::ArmorGimbalControlSpacing>& armors,
        const predictor::PredictTimeStamp& predict_time_stamp)
        : armors_(armors)
        , predict_time_stamp_(predict_time_stamp) { };
    ~PredictArmorInGimbalControl();

    void set(const std::vector<data::ArmorGimbalControlSpacing>& armors,
        const predictor::PredictTimeStamp& predict_time_stamp) {
        armors_             = armors;
        predict_time_stamp_ = predict_time_stamp;
    }

    const std::vector<data::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_;
    };

    const interfaces::ITimeStamped& GetTimeStamped() const override { return predict_time_stamp_; };

private:
    std::vector<data::ArmorGimbalControlSpacing> armors_;
    predictor::PredictTimeStamp predict_time_stamp_;
};
}