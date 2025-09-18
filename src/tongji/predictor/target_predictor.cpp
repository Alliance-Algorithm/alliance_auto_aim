#include "target_predictor.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <vector>

#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "target.hpp"

namespace world_exe::tongji::predictor {

std::vector<Target> ConstructTargets(const enumeration::ArmorIdFlag& id, std::time_t time_stamp) {
    for (int i = 0; i < 8; ++i) {
        auto car_id = static_cast<enumeration::ArmorIdFlag>(1 << i);
        if (!enumeration::IsFlagContains(id, car_id)) continue;

        bool is_balance = (car_id == enumeration::CarIDFlag::InfantryIII
            || car_id == enumeration::CarIDFlag::InfantryIV
            || car_id == enumeration::CarIDFlag::InfantryV);

        Eigen::VectorXd P0_dig(11);
        double radius;
        int armor_num;

        if (is_balance) {
            P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
            radius    = 0.2;
            armor_num = 2;
        } else if (car_id == enumeration::CarIDFlag::Outpost) {
            P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
            radius    = 0.2765;
            armor_num = 3;
        } else if (car_id == enumeration::CarIDFlag::Base) {
            P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
            radius    = 0.3205;
            armor_num = 3;
        } else {
            P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
            radius    = 0.2;
            armor_num = 4;
        }

        Eigen::Vector3d armor_xyz_in_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d armor_ypr_in_world = Eigen::Vector3d::Zero();
    }
}

class TargetPredictor::Impl {
public:
    Impl() { }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp) { }

    std::shared_ptr<interfaces::IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const {
    
    }

private:
    std::time_t last_update_time_stamp_ { 0 };
};

std ::shared_ptr<interfaces ::IArmorInGimbalControl> TargetPredictor::Predict(
    const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}

std ::shared_ptr<interfaces::IPredictor> TargetPredictor::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

TargetPredictor::TargetPredictor(const enumeration::ArmorIdFlag& id, std::time_t t)
    : pimpl_() { }
}