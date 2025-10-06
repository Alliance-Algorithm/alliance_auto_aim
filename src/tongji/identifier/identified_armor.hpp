#pragma once

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/time_stamped.hpp"
#include "util/index.hpp"
#include <cstdint>
#include <list>
#include <memory>

namespace world_exe::tongji::identifier {

struct SPArmor {
    const data::ArmorImageSpacing& armor;
    const cv::Point2f center;
    const int priority = 0;
};

class IdentifiedArmor final : public interfaces::IArmorInImage, public interfaces::ITimeStamped {
public:
    explicit IdentifiedArmor(const std::vector<data::ArmorImageSpacing>& armors) {
        for (const auto& armor : armors) {
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
    }

    const interfaces::ITimeStamped& GetTimeStamped() const override { return *this; }

    const std::time_t& GetTimeStamp() const override { return time_stamp_; };

    const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }

    std::shared_ptr<std::list<SPArmor>> get_sq_armor() {
        if (armor_list != nullptr) return armor_list;

        armor_list = std::make_shared<std::list<SPArmor>>();
        for (const auto& car : armors_)
            for (const auto& armor : car)
                armor_list->emplace_back(
                    armor, (armor.image_points[0] + armor.image_points[3]) / 2, 0);

        return armor_list;
    }

    static IdentifiedArmor DecorateIArmorInImage(const interfaces::IArmorInImage& armor) {
        throw std::runtime_error("Not implemented");
    }

    auto GetDetectedIDs() {
        enumeration::CarIDFlag result;
        for (int i = 0; i < 8; i++) {
            if (!armors_[i].empty()) {
                result = static_cast<enumeration::CarIDFlag>(
                    static_cast<uint32_t>(result) | static_cast<uint32_t>(armors_[i][0].id));
            }
        }
        return result;
    }

private:
    std::time_t time_stamp_ { 0 };
    std::array<std::vector<data::ArmorImageSpacing>, 8> armors_;
    std::shared_ptr<std::list<SPArmor>> armor_list = nullptr;
};
}