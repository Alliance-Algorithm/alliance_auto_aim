#pragma once

#include "enum/armor_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/time_stamped.hpp"
#include "tongji/time_stamp/time_stamp.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::identifier {

class IdentifiedArmor final : public interfaces::IArmorInImage {
public:
    explicit IdentifiedArmor(const std::vector<data::ArmorImageSpacing>& armors) {
        for (const auto& armor : armors) {
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return time_stamp_; }

    const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }

    static IdentifiedArmor DecorateIArmorInImage(const interfaces::IArmorInImage& armor) {
        throw std::runtime_error("Not implemented");
    }

private:
    time_stamp::TimeStamp time_stamp_ { std::chrono::steady_clock::now() };
    std::array<std::vector<data::ArmorImageSpacing>, 8> armors_;
};
}