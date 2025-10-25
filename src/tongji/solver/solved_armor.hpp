#pragma once

#include <array>
#include <chrono>
#include <ctime>

#include "interfaces/armor_in_camera.hpp"
#include "interfaces/time_stamped.hpp"
#include "tongji/time_stamp/time_stamp.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::solver {
class SolvedArmor final : public interfaces::IArmorInCamera {
public:
    explicit SolvedArmor(const std::vector<data::ArmorCameraSpacing>& armors)
        : time_stamp_(std::chrono::steady_clock::now()) {
        for (const auto& armor : armors) {
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
    }

    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }
    const interfaces::ITimeStamped& GetTimeStamped() const override { return time_stamp_; }

private:
    std::array<std::vector<data::ArmorCameraSpacing>, 8> armors_;
    time_stamp::TimeStamp time_stamp_;
};
}
