#include "target_snapshot_predictor.hpp"

#include <ctime>
#include <unordered_map>
#include <vector>

#include <Eigen/src/Geometry/Quaternion.h>
#include <opencv2/core/cvdef.h>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "enum/enum_tools.hpp"
#include "in_gimbal_control_armor.hpp"
#include "target.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {
class MultiTargetSnapshotPredictor::Impl {
public:
    Impl(const std::vector<Target>& targets)
        : targets_(targets)
        , last_time_stamp_(0) {
        id_ = enumeration::ArmorIdFlag::Unknow;
        for (const auto& target : targets_) {
            id_ = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(id_) | static_cast<uint32_t>(target.GetId()));
        }
    }

    const enumeration::ArmorIdFlag& GetId() const { return id_; }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(const std::time_t& time_stamp) {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            armor_map;
        const double dt = static_cast<double>(time_stamp - last_time_stamp_);

        for (auto& target : targets_) {
            Target iteration_target = target;
            iteration_target.Predict(dt);

            Eigen::VectorXd iteration_x = iteration_target.GetEkf_x();
            std::vector<data::ArmorGimbalControlSpacing> armors;

            for (int j = 0; j < iteration_target.GetArmorNum(); ++j) {
                auto angle = util::math::clamp_pm_pi(
                    iteration_x[6] + j * 2 * CV_PI / iteration_target.GetArmorNum());
                Eigen::Vector3d xyz = h_armor_xyz(iteration_x, j, iteration_target.GetArmorNum());
                data::ArmorGimbalControlSpacing armor;
                armor.id          = iteration_target.GetId();
                armor.position    = xyz;
                armor.orientation = util::math::euler_to_quaternion(angle, 15 / 180. * CV_PI, 0);
            }
            armor_map[iteration_target.GetId()] = armors;
        }
        return std::make_shared<InGimbalControlArmor>(armor_map, last_time_stamp_);
    }

private:
    std::vector<Target> targets_;
    enumeration::ArmorIdFlag id_;
    std::time_t last_time_stamp_;

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
};

MultiTargetSnapshotPredictor::MultiTargetSnapshotPredictor(const std::vector<Target>& targets)
    : pimpl_(std::make_unique<Impl>(targets)) { }

const enumeration ::ArmorIdFlag& MultiTargetSnapshotPredictor::GetId() const {
    return pimpl_->GetId();
}

std ::shared_ptr<interfaces::IArmorInGimbalControl> MultiTargetSnapshotPredictor::Predictor(
    const std ::time_t& time_stamp) const {
    return pimpl_->Predictor(time_stamp);
}
}