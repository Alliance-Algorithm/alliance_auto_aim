#pragma once

#include <unordered_set>
#include <utility>
#include <vector>

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::identifier {

class ArmorFilter {
public:
    explicit ArmorFilter()
        : invincible_armor_({ }) { }
    std::vector<data::ArmorImageSpacing> FilterArmor(
        std::vector<data::ArmorImageSpacing> armors) const {

        // 25赛季没有5号装甲板
        armors.erase(std::remove_if(armors.begin(), armors.end(),
            [](const auto& armor) { return armor.id == enumeration::ArmorIdFlag::InfantryV; }));
        // 不打前哨站
        armors.erase(std::remove_if(armors.begin(), armors.end(),
            [](const auto& armor) { return armor.id == enumeration::ArmorIdFlag::Outpost; }));
        // 过滤掉刚复活无敌的装甲板
        armors.erase(std::remove_if(armors.begin(), armors.end(),
                         [&](const auto& armor) { return invincible_armor_.count(armor.id); }),
            armors.end());

        return std::move(armors);
    }

    void Update(enumeration::CarIDFlag ids) {
        invincible_armor_.clear();
        for (auto id : util::enumeration::ExpandArmorIdFlags(ids)) {
            invincible_armor_.emplace(std::move(id));
        }
    }

private:
    std::unordered_set<enumeration::ArmorIdFlag> invincible_armor_;
};
}