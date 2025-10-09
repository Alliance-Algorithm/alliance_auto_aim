#include "decider.hpp"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "tongji/decider/armor_info.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::decider {

class Decider::Impl {
public:
    explicit Impl(PriorityMode mode = PriorityMode::MODE_ONE)
        : mode_(mode) { }

    void SetInvincibleArmors(const enumeration::ArmorIdFlag& armors) {
        invincible_armor_.clear();
        for (const auto id : util::enumeration::ExpandArmorIdFlags(armors)) {
            invincible_armor_.emplace(std::move(armors));
        }
    }

    void SetPriority( std::vector<ArmorInfo>& detected_result) const {
        const PriorityMap& priority_map = (mode_ == PriorityMode::MODE_ONE) ? mode1 : mode2;
        if (!detected_result.empty()) {
            for (auto& armor : detected_result) {
                armor.priority = priority_map.at(armor.armor_spacing.id);
            }
        }
    }

    enumeration::ArmorIdFlag GetSortedArmor(std::vector<ArmorInfo>& armors) const {
        if (armors.empty()) return enumeration::ArmorIdFlag::None;

        // 对每个 DetectionResult 调用 armor_filter 和 set_priority
        // ArmorFilter(armors);
        // SetPriority(armors);

        // 对每个 DetectionResult 中的 armors 进行排序
        cv::Point2d img_center(1440.0 / 2, 1080.0 / 2); // TODO
        std::sort(armors.begin(), armors.end(), [&](const auto& a, const auto& b) {
            auto center_a =
                (a.armor_spacing.image_points[0] + a.armor_spacing.image_points[3]) * 0.5f;
            auto center_b =
                (b.armor_spacing.image_points[0] + b.armor_spacing.image_points[3]) * 0.5f;
            return cv::norm(center_a - img_center) < cv::norm(center_b - img_center);
        });

        std::sort(armors.begin(), armors.end(),
            [](const auto& a, const auto& b) { return a.priority < b.priority; });
        return armors.front().armor_spacing.id;
    }

    bool ArmorFilter(std::vector<ArmorInfo>& armors) const {
        if (armors.empty()) return true;

        // 25赛季没有5号装甲板
        armors.erase(std::remove_if(armors.begin(), armors.end(),
            [](const auto& armor) { return armor.armor_spacing.id == ArmorId::InfantryV; }));
        // 不打前哨站
        armors.erase(std::remove_if(armors.begin(), armors.end(),
            [](const auto& armor) { return armor.armor_spacing.id == ArmorId::Outpost; }));
        // 过滤掉刚复活无敌的装甲板
        armors.erase(
            std::remove_if(armors.begin(), armors.end(),
                [&](const auto& armor) { return invincible_armor_.count(armor.armor_spacing.id); }),
            armors.end());

        return armors.empty();
    }

private:
    using ArmorPriority = world_exe::tongji::decider::ArmorPriority;
    using ArmorId       = world_exe::enumeration::ArmorIdFlag;
    using ArmorInfo     = world_exe::tongji::decider::ArmorInfo;

    using PriorityMap = std::unordered_map<ArmorId, ArmorPriority>;

    const PriorityMap mode1 = {
        { ArmorId::Hero, ArmorPriority::Second },       //
        { ArmorId::Engineer, ArmorPriority::Forth },    //
        { ArmorId::InfantryIII, ArmorPriority::First }, //
        { ArmorId::InfantryIV, ArmorPriority::First },  //
        { ArmorId::InfantryV, ArmorPriority::Third },   //
        { ArmorId::Sentry, ArmorPriority::Third },      //
        { ArmorId::Outpost, ArmorPriority::Fifth },     //
        { ArmorId::Base, ArmorPriority::Fifth },        //
        { ArmorId::Unknow, ArmorPriority::Fifth }       //
    };

    const PriorityMap mode2 = {
        { ArmorId::Hero, ArmorPriority::Second },       //
        { ArmorId::Engineer, ArmorPriority::Forth },    //
        { ArmorId::InfantryIII, ArmorPriority::First }, //
        { ArmorId::InfantryIV, ArmorPriority::First },  //
        { ArmorId::InfantryV, ArmorPriority::Third },   //
        { ArmorId::Sentry, ArmorPriority::Third },      //
        { ArmorId::Outpost, ArmorPriority::Fifth },     //
        { ArmorId::Base, ArmorPriority::Fifth },        //
        { ArmorId::Unknow, ArmorPriority::Fifth }       //
    };

    std::unordered_set<ArmorId> invincible_armor_;
    PriorityMode mode_;
};

Decider::Decider(PriorityMode mode)
    : pimpl_(std::make_unique<Impl>(mode)) { }

void Decider::SetInvincibleArmors(const enumeration::ArmorIdFlag& armors) {
    return pimpl_->SetInvincibleArmors(armors);
}

void Decider::SetPriority( std::vector<ArmorInfo>& detected_result) const {
    return pimpl_->SetPriority(detected_result);
}

bool Decider::ArmorFilter(std::vector<ArmorInfo>& armors) { return pimpl_->ArmorFilter(armors); }

enumeration::CarIDFlag Decider::GetSortedArmor(std::vector<ArmorInfo>& armors) const {
    return pimpl_->GetSortedArmor(armors);
}

Decider::~Decider() = default; 
}
