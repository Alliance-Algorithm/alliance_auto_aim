#include "predictor_record.hpp"

#include <ctime>
#include <memory>
#include <unordered_map>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "in_gimbal_control_armor.hpp"

namespace world_exe::tongji::predictor {
class PredictorRecord::Impl {
public:
    Impl(const enumeration::ArmorIdFlag& id,
        const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> snapshots)
        : id_(id)
        , snapshots_(snapshots) { }

    const enumeration::ArmorIdFlag& GetId() const { return id_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(const std::time_t& time_stamp) {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        for (int i = 0; i < 8; i++) {
            auto decomposed_id = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);
            if (enumeration::IsFlagContains(id_, decomposed_id)) {
                auto it = snapshots_.find(decomposed_id);
                if (it == snapshots_.end()) continue;

                auto copy = std::make_shared<Target>(*(it->second));
                copy->Predict(time_stamp);
                result[decomposed_id] = copy->GetArmorGimbalControlSpacings();
            }
        }

        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

private:
    std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>> snapshots_;
    enumeration::ArmorIdFlag id_;
};

PredictorRecord::PredictorRecord(const enumeration::ArmorIdFlag& id,
    const std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<Target>>& snapshots)
    : pimpl_(std::make_unique<Impl>(id, snapshots)) { }
PredictorRecord::~PredictorRecord() = default;

const enumeration ::ArmorIdFlag& PredictorRecord::GetId() const { return pimpl_->GetId(); }

std ::shared_ptr<interfaces::IArmorInGimbalControl> PredictorRecord::Predictor(
    const std ::time_t& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}
}