#include "target_predictor.hpp"

#include <memory>
#include <unordered_map>
#include <utility>

#include "enum/enum_tools.hpp"
#include "tongji/predictor/in_gimbal_control_armor.hpp"
#include "tongji/predictor/predictor_record.hpp"
#include "tongji/predictor/target.hpp"

namespace world_exe::tongji::predictor {

class TargetPredict::Impl {
public:
    Impl(std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> targets) { }
    void RegisterTarget(enumeration::ArmorIdFlag id, const Target& target) {
        targets_[id] = std::make_shared<Target>(target);
    }

    void RemoveTarget(enumeration::ArmorIdFlag id) { }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp) {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        for (int i = 0; i < 8; i++) {
            auto decomposed_id = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);
            if (enumeration::IsFlagContains(id, decomposed_id)) {
                auto it = targets_.find(decomposed_id);
                if (it == targets_.end()) continue;

                it->second->Predict(time_stamp);
                result[decomposed_id] = it->second->GetArmorGimbalControlSpacings();
            }
        }

        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

    std::shared_ptr<interfaces::IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const {
        std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> snapshot_map;

        for (int i = 0; i < 8; i++) {
            auto decomposed_id = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);
            if (enumeration::IsFlagContains(id, decomposed_id)) {

                auto it = targets_.find(decomposed_id);
                if (it == targets_.end()) continue;

                snapshot_map[decomposed_id] = std::make_shared<Target>(*(it->second));
            }
        }
        return std::make_shared<PredictorRecord>(id, snapshot_map);
    }

private:
    std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> targets_;
};

TargetPredict::TargetPredict(
    std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> targets)
    : pimpl_(std::make_unique<Impl>(std::move(targets))) { }
TargetPredict::~TargetPredict() = default;

std ::shared_ptr<interfaces ::IArmorInGimbalControl> TargetPredict::Predict(
    const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}

std ::shared_ptr<interfaces::IPredictor> TargetPredict::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

}