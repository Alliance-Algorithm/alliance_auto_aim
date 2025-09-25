#pragma once

#include <array>
#include <ctime>

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/time_stamped.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::solver {
class SolvedArmor final : public interfaces::IArmorInCamera, public interfaces::ITimeStamped {
public:
    explicit SolvedArmor(const std::vector<data::ArmorCameraSpacing>& armors) {
        for (const auto& armor : armors) {
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
        time_stamp_ = 0;
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return *this; }

    const std::time_t& GetTimeStamp() const override { return time_stamp_; };

    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }

private:
    std::time_t time_stamp_ { 0 };
    std::array<std::vector<data::ArmorCameraSpacing>, 8> armors_;
};
}