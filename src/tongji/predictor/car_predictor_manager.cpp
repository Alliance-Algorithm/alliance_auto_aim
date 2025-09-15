#include "car_predictor_manager.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "tongji/predictor/car_predictor.hpp"
#include "tongji/predictor/in_gimbal_control_armor.hpp"
#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::predictor {
class CarPredictorManager::Impl {
public:
    Impl() = default;

    const enumeration ::CarIDFlag& GetId() {
        if (ids_dirty_) {
            ids_ = enumeration::CarIDFlag::None;
            for (const auto& [id, _] : predictors_) {
                ids_ = static_cast<enumeration::CarIDFlag>(
                    static_cast<uint32_t>(ids_) | static_cast<uint32_t>(id));
            }
            ids_dirty_ = false;
        }
        return ids_;
    }

    auto Predictor(const std ::time_t& time_stamp) {
        InGimbalControlArmor target_armors;
        for (auto& predictor : predictors_) {
            target_armors.Add(
                predictor.first, predictor.second.GetPredictedArmors(predictor.first, time_stamp));
        }
        return std::make_shared<InGimbalControlArmor>(target_armors);
    }

    void Tick(const TimeStamp& now) {
        std::vector<enumeration::CarIDFlag> to_remove;

        for (auto& [id, predictor] : predictors_) {
            predictor.PredictTo(now);
            predictor.TickStatus();

            if (predictor.IsLost()) {
                to_remove.push_back(id);
            }
        }
        // logger()->info("Removing lost predictor: {}", static_cast<int>(id));
        for (auto id : to_remove) {
            predictors_.erase(id);
        }
    }

    void SetPredictorBySingleID(const enumeration::CarIDFlag& id,
        const Eigen::Vector3d& armor_xyz_in_world, const Eigen::Vector3d& armor_ypr_in_world,
        std::time_t init_time_stamp) {

        auto it = predictors_.find(id);
        if (it != predictors_.end()) {
            it->second.Reset(armor_xyz_in_world, armor_ypr_in_world, init_time_stamp);
        } else {
            bool is_balance = (id == enumeration::CarIDFlag::InfantryIII
                | id == enumeration::CarIDFlag::InfantryIV
                | id == enumeration::CarIDFlag::InfantryV);

            Eigen::VectorXd P0_dig(11);
            double radius;
            int armor_num;

            if (is_balance) {
                P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
                radius    = 0.2;
                armor_num = 2;
            } else if (id == enumeration::CarIDFlag::Outpost) {
                P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
                radius    = 0.2765;
                armor_num = 3;
            } else if (id == enumeration::CarIDFlag::Base) {
                P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
                radius    = 0.3205;
                armor_num = 3;
            } else {
                P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
                radius    = 0.2;
                armor_num = 4;
            }

            predictors_.emplace(id,
                CarPredictor { id, armor_xyz_in_world, armor_ypr_in_world, init_time_stamp, radius,
                    armor_num, P0_dig });
            ids_dirty_ = true;
        }
    }

    void RemovePredictorBySingleID(const enumeration::CarIDFlag& id) {
        predictors_.erase(id);
        ids_dirty_ = true;
    }

    bool HasPredictor(const enumeration::CarIDFlag& id) const {
        return predictors_.find(id) != predictors_.end();
    }

private:
    enumeration::CarIDFlag ids_ { enumeration::CarIDFlag::None };
    std::unordered_map<enumeration::CarIDFlag, CarPredictor> predictors_;
    mutable bool ids_dirty_ = true;
};

const enumeration ::ArmorIdFlag& CarPredictorManager::GetId() const { return pimpl_->GetId(); }

std::shared_ptr<interfaces::IArmorInGimbalControl> CarPredictorManager::Predictor(
    const std ::time_t& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}

void CarPredictorManager::SetPredictorBySingleID(const enumeration::CarIDFlag& id,
    Eigen::Vector3d armor_xyz_in_world, Eigen::Vector3d armor_ypr_in_world, time_t time_stamp) {
    return pimpl_->SetPredictorBySingleID(id, armor_xyz_in_world, armor_ypr_in_world, time_stamp);
}

void CarPredictorManager::SetPredictorBySingleID(
    const enumeration::CarIDFlag& id, time_t time_stamp) {
    return pimpl_->SetPredictorBySingleID(
        id, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), time_stamp);
}

CarPredictorManager::CarPredictorManager()
    : pimpl_(std::make_unique<Impl>()) { }

}
