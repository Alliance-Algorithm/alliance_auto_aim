#pragma once

#include <vector>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "tongji/predictor/time_stamp.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::predictor {

class InGimbalControlArmor final : public interfaces::IArmorInGimbalControl {
public:
    InGimbalControlArmor() = default;
    InGimbalControlArmor(const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
        TimeStamp time_stamp)
        : armors_(armors)
        , time_stamp_(time_stamp) { }

    const std ::vector<data ::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration ::ArmorIdFlag& armor_id) const override {
        auto armor_id_index = util::enumeration::GetIndex(armor_id);
        return armors_.at(armor_id_index);
    }

    void Set(const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
        const predictor::TimeStamp& predict_time_stamp) {
        armors_     = armors;
        time_stamp_ = predict_time_stamp;
    }

    void Add(
        enumeration::CarIDFlag car_id, const std::vector<data::ArmorGimbalControlSpacing>& armors) {
        auto& target_vector = armors_.at(util::enumeration::GetIndex(car_id));
        target_vector.insert(target_vector.end(), armors.begin(), armors.end());
    }

    void Clear() {
        for (auto& vec : armors_) {
            vec.clear();
        }
    }

    void SetArmorsByCarId(
        const std::vector<data::ArmorGimbalControlSpacing>& armors, TimeStamp time_stamp) {
        if (armors.empty()) return;
        armors_[util::enumeration::GetIndex(armors.at(0).id)] = armors;
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return time_stamp_; }

    void SetTimeStamp(const TimeStamp time_stamp) { time_stamp_ = time_stamp; }

private:
    TimeStamp time_stamp_;
    std::array<std::vector<data::ArmorGimbalControlSpacing>, 8> armors_;
};
}