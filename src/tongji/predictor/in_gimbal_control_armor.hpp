#pragma once

#include <unordered_map>
#include <vector>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::predictor {

class InGimbalControlArmor final : public interfaces::IArmorInGimbalControl {
public:
    InGimbalControlArmor(
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            armors,
        TimeStamp time_stamp)
        : armors_(armors)
        , time_stamp_(time_stamp) { }

    const std ::vector<data ::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration ::ArmorIdFlag& armor_id) const override {
        static const std::vector<data::ArmorGimbalControlSpacing> empty;
        auto it = armors_.find(armor_id);
        return it != armors_.end() ? it->second : empty;
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return time_stamp_; }

private:
    TimeStamp time_stamp_;
    std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
        armors_;
};
}