#pragma once

#include <ctime>
#include <unordered_map>
#include <vector>

#include "data/armor_gimbal_control_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"

namespace world_exe::tongji::predictor {

class InGimbalControlArmor final : public interfaces::IArmorInGimbalControl {
public:
    InGimbalControlArmor(const std::unordered_map<enumeration::ArmorIdFlag,
                         std::vector<data::ArmorGimbalControlSpacing>>& all_armors,
                         const data::TimeStamp& time_stamp)
        : armors_map_(std::move(all_armors))
        , time_stamp_(time_stamp) { }

    const std ::vector<data ::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration ::ArmorIdFlag& armor_id) const override {
        auto it = armors_map_.find(armor_id);
        return it != armors_map_.end() ? it->second : empty;
    }

    const data::TimeStamp& GetTimeStamp() const override { return time_stamp_; }

private:
    data::TimeStamp time_stamp_;
    std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
        armors_map_;
    static inline const std::vector<data::ArmorGimbalControlSpacing> empty={};
};
}