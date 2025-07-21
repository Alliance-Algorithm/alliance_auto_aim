#pragma once

#include "interfaces/armor_in_image.hpp"
#include "identifier_time_stamp.hpp"
#include "util/index.hpp"

namespace world_exe::identifier {
class IdentifierArmor : public interfaces::IArmorInImage {
public:
    IdentifierArmor()=default;
    IdentifierArmor(const std::vector<data::ArmorImageSpacing>& armors) {
        for (const auto armor : armors)
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return time_stamp_; }

    void SetArmors(const std::vector<data::ArmorImageSpacing>& armors) {
        for (auto& armors : armors_)
            armors.clear();
        for (const auto armor : armors)
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
    }

    const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }

private:
    IdentifierTimeStamp time_stamp_;
    std::array<std::vector<data::ArmorImageSpacing>, 8> armors_;
};
}