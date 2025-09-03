#pragma once

#include "enum/armor_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/time_stamped.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::identifier {
class IdentifierArmor final : public interfaces::IArmorInImage, public interfaces::ITimeStamped {
public:
    explicit IdentifierArmor(const std::vector<data::ArmorImageSpacing>& armors) {
        SetArmors(armors);
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return *this; }

    const std::time_t& GetTimeStamp() const override { return time_stamp_; };

    void SetArmors(const std::vector<data::ArmorImageSpacing>& armors) {
        std::array<std::vector<data::ArmorImageSpacing>, 8> temp_armors;
        for (const auto& armor : armors) {
            temp_armors[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
        armors_ = std::move(temp_armors);
    }

    const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }
    
private:
    std::time_t time_stamp_ { 0 };
    std::array<std::vector<data::ArmorImageSpacing>, 8> armors_;
};
}