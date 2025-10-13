#include "live_target_manager.hpp"

#include <cstdint>
#include <ctime>
#include <memory>
#include <unordered_map>

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "tongji/predictor/in_gimbal_control_armor.hpp"
#include "tongji/predictor/live_target.hpp"
#include "tongji/predictor/target_snapshot_manager.hpp"
#include "tongji/predictor/tracker.hpp"
#include "util/index.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

class LiveTargetManager::Impl {
public:
    Impl(const double& time_delay, const double& yaw_offset, const double& pitch_offset,
        double timeout_sec = 0.1)
        : targets_map_()
        , tracker_(std::make_unique<predictor::Tracker>())
        , last_update_timestamp_(std::time(nullptr))
        , tracking_id_(enumeration::CarIDFlag::None)
        , time_delay_(time_delay)
        , yaw_offset_(yaw_offset)
        , pitch_offset_(pitch_offset)
        , timeout_sec_(timeout_sec) { }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& flag, const std::time_t& time_stamp) {
        std::unordered_map<enumeration::ArmorIdFlag, std::vector<data::ArmorGimbalControlSpacing>>
            result;

        for (auto id : util::enumeration::ExpandArmorIdFlags(flag)) {
            auto it = targets_map_.find(id);
            if (it != targets_map_.end() && it->second && it->second->IsConverged()) {
                auto spacings = it->second->GetArmorGimbalControlSpacings();
                result[id]    = spacings;
            }
        }

        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    } // TODO: ** 目前 ** 我认为这个函数是多余的

    std::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration::ArmorIdFlag& flag) const {

        if (targets_map_.empty()) return nullptr; // TODO

        return std::make_shared<TargetSnapshotManager>(
            flag, targets_map_, last_update_timestamp_, bullet_speed_, yaw_offset_, pitch_offset_);
    }

    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
        const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image, const std::time_t& now,
        const double& bullet_speed) {

        UpdateTimeStamp(data->GetTimeStamped().GetTimeStamp());
        UpdateTargetMap(data, now);
        UpdateTarget(data, armors_in_image, now);
        UpdateBulletSpeed(bullet_speed);
    }

    auto GetAllowedTargetID() const -> enumeration::CarIDFlag const {
        if (targets_map_.at(tracking_id_)->IsConverged()) {
            return tracking_id_;
        }
        return enumeration::CarIDFlag::None;
    }

private:
    void UpdateBulletSpeed(const double& bullet_speed) { bullet_speed_ = bullet_speed; }
    void UpdateTimeStamp(const time_t& time_stamp) { last_update_timestamp_ = time_stamp; }
    void UpdateTargetMap(
        std::shared_ptr<interfaces::IPreDictorUpdatePackage> data, const std::time_t& now) {
        const Eigen::Affine3d transform       = data->GetTransform();
        const Eigen::Matrix3d rotation_matrix = transform.linear();
        const auto armors_interface           = data->GetArmors();

        targets_map_.clear();
        for (int i; i < static_cast<int>(enumeration::CarIDFlag::Count); i++) {
            auto id = static_cast<enumeration::CarIDFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);

            const auto& armors_list = armors_interface->GetArmors(id);
            if (armors_list.empty()) return;

            const auto& armor                  = armors_list.front();
            const Eigen::Vector3d xyz_in_world = transform * armor.position;
            const Eigen::Vector3d ypr_in_world = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
            targets_map_[id] =
                std::move(std::make_shared<LiveTarget>(xyz_in_world, ypr_in_world, id));
        }
    }

    void UpdateTarget(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
        const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image, const std::time_t& now) {
        const Eigen::Affine3d transform       = data->GetTransform();
        const Eigen::Matrix3d rotation_matrix = transform.linear();
        const auto armors_interface           = data->GetArmors();

        tracking_id_ = tracker_->SelectTrackingTargetID(armors_in_image, now);

        const auto& armors_list = armors_interface->GetArmors(tracking_id_);
        if (armors_list.empty()) return;

        const auto& armor                  = armors_list.front();
        const Eigen::Vector3d xyz_in_world = transform * armor.position;
        const Eigen::Vector3d ypr_in_world = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
        targets_map_[tracking_id_]->Update(static_cast<double>(now - last_update_timestamp_),
            xyz_in_world, ypr_in_world, util::math::xyz2ypd(xyz_in_world));
    }

    std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>> targets_map_;
    std::unique_ptr<predictor::Tracker> tracker_;
    std::time_t last_update_timestamp_;
    enumeration::CarIDFlag tracking_id_;

    double bullet_speed_;
    const double time_delay_;
    const double yaw_offset_;
    const double pitch_offset_;

    const double timeout_sec_;
};

LiveTargetManager::LiveTargetManager(const double& time_delay, const double& yaw_offset,
    const double& pitch_offset, double timeout_sec)
    : pimpl_(std::make_unique<Impl>(time_delay, yaw_offset, pitch_offset, timeout_sec)) { }
LiveTargetManager::~LiveTargetManager() = default;

std ::shared_ptr<interfaces ::IArmorInGimbalControl> LiveTargetManager::Predict(
    const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}
std ::shared_ptr<interfaces::IPredictor> LiveTargetManager::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

void LiveTargetManager::Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
    const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image, const std::time_t& now,
    const double& bullet_speed) {
    return pimpl_->Update(data, armors_in_image, now, bullet_speed);
}
auto LiveTargetManager::GetAllowedTargetID() const -> enumeration::ArmorIdFlag const {
    return pimpl_->GetAllowedTargetID();
}
}