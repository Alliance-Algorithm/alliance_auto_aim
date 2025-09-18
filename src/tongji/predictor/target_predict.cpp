#include "target_predict.hpp"

#include <list>
#include <memory>
#include <unordered_map>

#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "tongji/predictor/in_gimbal_control_armor.hpp"
#include "tongji/predictor/target.hpp"
#include "tongji/predictor/target_snapshot_predictor.hpp"

namespace world_exe::tongji::predictor {

class TargetPredict::Impl {
public:
    Impl(std::list<Target> targets) {
        for (auto target : targets) {
            targets_map_.insert({ target.GetId(), target });
        }
    }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp) {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            armor_map;

        const auto& dt   = time_stamp - last_time_stamp_;
        last_time_stamp_ = time_stamp;

        for (auto& target_value : targets_map_) {
            target_value.second.Predict(dt);
            auto x = target_value.second.GetEkf_x();

            for (int j = 0; j < target_value.second.GetArmorNum(); ++j) {
                auto angle = util::math::clamp_pm_pi(
                    x[6] + j * 2 * CV_PI / target_value.second.GetArmorNum());
                Eigen::Vector3d xyz = h_armor_xyz(x, j, target_value.second.GetArmorNum());
                data::ArmorGimbalControlSpacing armor;
                armor.id          = target_value.second.GetId();
                armor.position    = xyz;
                armor.orientation = util::math::euler_to_quaternion(angle, 15 / 180. * CV_PI, 0);
            }
        }
        return std::make_shared<InGimbalControlArmor>(armor_map, last_time_stamp_);
    }

    std::shared_ptr<interfaces::IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const {
        std::vector<Target> targets;
        for (int i; i < 8; i++) {
            const auto& car_id = static_cast<enumeration::ArmorIdFlag>(1 << i);
            if (enumeration::IsFlagContains(id, car_id)) {
                targets.emplace_back(ConstructTargets(car_id, )); // TODO
            }
        }
        return std::make_shared<MultiTargetSnapshotPredictor>(targets);
    }

private:
    std::vector<Target> ConstructTargets(
        const enumeration::ArmorIdFlag& id, std::time_t& time_stamp) const {
        std::vector<Target> targets;
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
            // 不知道这个armor_xyz_in_world、armor_ypr_in_world（可以理解为x）可以从哪里传入
            Eigen::Vector3d armor_xyz_in_world = Eigen::Vector3d::Zero();
            Eigen::Vector3d armor_ypr_in_world = Eigen::Vector3d::Zero();
            targets.emplace_back(Target(armor_xyz_in_world, armor_ypr_in_world, car_id, time_stamp,
                radius, armor_num, P0_dig));
        }
        return targets;
    }

    // 计算出装甲板中心的坐标（考虑长短轴）
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd& x, int id, int armor_num) const {
        auto angle   = util::math::clamp_pm_pi(x[6] + id * 2 * CV_PI / armor_num);
        auto use_l_h = (armor_num == 4) && (id == 1 || id == 3);

        auto r       = (use_l_h) ? x[8] + x[9] : x[8];
        auto armor_x = x[0] - r * std::cos(angle);
        auto armor_y = x[2] - r * std::sin(angle);
        auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

        return { armor_x, armor_y, armor_z };
    }

    std::time_t last_time_stamp_ { 0 };
    std::unordered_map<enumeration::CarIDFlag, Target> targets_map_;
};

}