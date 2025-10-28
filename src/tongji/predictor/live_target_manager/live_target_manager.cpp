#include "live_target_manager.hpp"

#include <cstdint>
#include <ctime>
#include <memory>
#include <unordered_map>

#include "../in_gimbal_control_armor.hpp"
#include "../target_snapshot_manager/target_snapshot_manager.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "live_target.hpp"
#include "tracker.hpp"
#include "util/index.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

class LiveTargetManager::Impl {
public:
    Impl(const std::string& config_path, const double& timeout_sec)
        : targets_map_()
        , tracker_(std::make_unique<predictor::Tracker>())
        , last_update_timestamp_(std::time(nullptr))
        , tracking_id_(enumeration::CarIDFlag::None)
        , config_path_(config_path) { }

    /*
    不懂为什么要实现这个接口，不需要这个返回值
    std::shared_ptr<interfaces::IArmorInGimbalControl>，吧？

    原因是：卡尔曼滤波器 更改状态变量x的值 的操纵
    写在了update函数中，这里的predict是没有副作用的预测，
    但是又没考虑飞行时间，理论上来说，误差更大了，
    需要得到的是考虑飞行时间得到的  std::shared_ptr<interfaces::IArmorInGimbalControl>
    所以目前认为它是多余的
    */

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
    }

    /*
    猜测接口的意思是通过外界传入id来 获得对应的预测器（副本），
    而从外界传入的id是从  const CarIDFlag GetAttackCarId() const这类接口中传入的，
    但是我具体的id已经存在了targets_map_中，无需从外部传入

    哦，原来是这样，怪不得不知道传啥参数进去
    */
    std::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration::ArmorIdFlag& flag) const {

        if (targets_map_.empty()) return nullptr;

        return std::make_shared<TargetSnapshotManager>(
            config_path_, flag, targets_map_, last_update_timestamp_);
    }
    // 为何传递了一个time_t 给double
    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
        const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image, const double& dt) {

        UpdateTimeStamp(data->GetTimeStamped().GetTimeStamp());
        UpdateTargetMap(data);
        UpdateTarget(data, armors_in_image, dt);
    }

    auto GetAllowedTargetID() const -> enumeration::CarIDFlag const {
        if (targets_map_.at(tracking_id_)->IsConverged()) {
            return tracking_id_;
        }
        return enumeration::CarIDFlag::None;
    }

private:
    void UpdateTimeStamp(const time_t& time_stamp) { last_update_timestamp_ = time_stamp; }
    void UpdateTargetMap(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data) {
        const Eigen::Affine3d transform       = data->GetTransform();
        const Eigen::Matrix3d rotation_matrix = transform.rotation();
        const auto armors_interface           = data->GetArmors();

        targets_map_.clear();
        for (int i; i < static_cast<int>(enumeration::CarIDFlag::Count); i++) {
            auto id = static_cast<enumeration::CarIDFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);

            const auto& armors_list = armors_interface->GetArmors(id);
            if (armors_list.empty()) return;

            const auto& armor                   = armors_list.front();
            const Eigen::Vector3d xyz_in_gimbal = transform * armor.position;
            const Eigen::Vector3d ypr_in_gimbal = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
            targets_map_[id] = std::make_shared<LiveTarget>(xyz_in_gimbal, ypr_in_gimbal, id);
        }
    }

    void UpdateTarget(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
        const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image, const double& dt) {
        const Eigen::Affine3d transform       = data->GetTransform();
        const Eigen::Matrix3d rotation_matrix = transform.linear();
        const auto armors_interface           = data->GetArmors();

        tracking_id_ = tracker_->SelectTrackingTargetID(armors_in_image);

        const auto& armors_list = armors_interface->GetArmors(tracking_id_);
        if (armors_list.empty()) return;

        const auto& armor                   = armors_list.front();
        const Eigen::Vector3d xyz_in_gimbal = transform * armor.position;
        const Eigen::Vector3d ypr_in_gimbal = rotation_matrix.eulerAngles(2, 1, 0); // ZYX
        targets_map_[tracking_id_]->Update(
            dt, xyz_in_gimbal, ypr_in_gimbal, util::math::xyz2ypd(xyz_in_gimbal));
    }

    std::unordered_map<enumeration::ArmorIdFlag, std::shared_ptr<LiveTarget>> targets_map_;
    std::unique_ptr<predictor::Tracker> tracker_;
    std::time_t last_update_timestamp_;
    enumeration::CarIDFlag tracking_id_;

    const std::string config_path_;
};

LiveTargetManager::LiveTargetManager(const std::string& config_path, const double& timeout_sec)
    : pimpl_(std::make_unique<Impl>(config_path, timeout_sec)) { }
LiveTargetManager::~LiveTargetManager() = default;

std ::shared_ptr<interfaces ::IArmorInGimbalControl> LiveTargetManager::Predict(
    const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}
std ::shared_ptr<interfaces::IPredictor> LiveTargetManager::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

static constexpr double cast_nanosec_sec(const long nanosec) {return nanosec * 1e-9;}

void LiveTargetManager::Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data,
    const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image) {
    return pimpl_->Update(data, armors_in_image, cast_nanosec_sec(data->GetTimeStamped().GetTimeStamp()));
}
// auto LiveTargetManager::GetAllowedTargetID() const -> enumeration::ArmorIdFlag const {
//     return pimpl_->GetAllowedTargetID();
// }
}
