#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core/types.hpp>
#include <vector>

#include "../predictor/target_snapshot.hpp"
#include "../predictor/target_snapshot_manager.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"
#include "tongji/decider/armor_info.hpp"
#include "tongji/decider/decider.hpp"
#include "tongji/fire_controller/aim_point_chooser.hpp"
#include "tongji/identifier/identified_armor.hpp"
#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::fire_control {

enum class TrackState {
    Lost,      //
    Detecting, //
    Tracking,  //
    TempLost,  //
    Switching  //
};

class Tracker final {
    using TargetSnapshotManager = world_exe::tongji::predictor::TargetSnapshotManager;
    using TargetSnapshot        = world_exe::tongji::predictor::TargetSnapshot;
    using ArmorInImage          = world_exe::tongji::identifier::IdentifiedArmor;
    using EnemiesState          = world_exe::interfaces::ICarState;

public:
    Tracker()
        : last_timestamp_(std::time(nullptr))
        , decider_(std::make_unique<decider::Decider>()) { }

    ~Tracker() = default;

    auto SelectTrackingTarget(std::shared_ptr<interfaces::IArmorInImage> armors_in_image,
        const std::shared_ptr<TargetSnapshotManager>& snapshot_manager_) noexcept
        -> std::unique_ptr<TargetSnapshot> {

        auto detected_ids    = enumeration::CarIDFlag::None;
        auto armor_info_list = std::vector<decider::ArmorInfo> { };

        for (uint32_t i = 0; i < static_cast<uint32_t>(CarIDFlag::Count); ++i) {
            auto id = static_cast<CarIDFlag>(static_cast<uint32_t>(CarIDFlag::Hero) << i);

            if (armors_in_image->GetArmors(id).empty()) continue;

            detected_ids = static_cast<CarIDFlag>(
                static_cast<uint32_t>(detected_ids) | static_cast<uint32_t>(id));

            for (const auto& armor : armors_in_image->GetArmors(id)) {
                auto armor_info = decider::ArmorInfo(armor);
                armor_info_list.emplace_back(armor_info);
            }
        }

        decider_->SetInvincibleArmors(invincible_armors_);
        auto is_empty = decider_->ArmorFilter(armor_info_list);
        decider_->SetPriority(armor_info_list);

        auto sorted_id = decider_->GetSortedArmor(armor_info_list);
        if (sorted_id == enumeration::ArmorIdFlag::None) {
            return nullptr;
        }

        auto snapshot = snapshot_manager_->GetSingleSnapshot(sorted_id);

        if (state_ == TrackState::Tracking) {
            SetState(TrackState::Switching);
            temp_lost_count_ = 0;
        }

        auto now = predictor::TimeStamp(std::time(nullptr));
        if (state_ != TrackState::Lost && now.SecondsSince(last_timestamp_) > 0.1) {
            SetState(TrackState::Lost);
            ResetTracking();
            return nullptr;
        }

        last_timestamp_  = now;
        tracking_car_id_ = snapshot->GetID();
        return snapshot;

        ResetTracking();
        return nullptr;
    }

    void SetInvincibleArmors(const CarIDFlag& invincible_armors) {
        invincible_armors_ = invincible_armors;
    }

    world_exe::enumeration::CarIDFlag GetCurrentTargetID() const { return tracking_car_id_; }

    void UpdateState(bool found) {
        switch (state_) {
        case TrackState::Lost: {
            if (found) {
                SetState(TrackState::Detecting);
                detect_count_++;
            } else {
                ResetTracking();
            }
            break;
        }

        case TrackState::Detecting: {
            if (found) {
                detect_count_++;
                if (detect_count_ >= min_detect_count_) SetState(TrackState::Tracking);
            } else {
                detect_count_ = 0;
                SetState((pre_state_ == TrackState::Switching) ? TrackState::Switching
                                                               : TrackState::Lost);
            }
            break;
        }

        case TrackState::Tracking: {
            if (!found) {
                temp_lost_count_ = 1;
                SetState(TrackState::TempLost);
            }
            break;
        }

        case TrackState::Switching: {
            if (found) {
                SetState(TrackState::Detecting);
            } else {
                temp_lost_count_++;
                if (temp_lost_count_ > max_switch_count_) {
                    SetState(TrackState::Lost);
                    ResetTracking();
                };
            }
            break;
        }

        case TrackState::TempLost: {
            if (found) {
                SetState(TrackState::Tracking);
            } else {
                temp_lost_count_++;
                max_temp_lost_count_ = (tracking_car_id_ == enumeration::ArmorIdFlag::Outpost)
                    ? outpost_max_temp_lost_count_
                    : normal_max_temp_lost_count_;

                if (temp_lost_count_ > max_temp_lost_count_) {
                    SetState(TrackState::Lost);
                    ResetTracking();
                };
            }
            break;
        }
        }
    }

    TrackState GetState() const { return state_; }

private:
    void SetState(TrackState new_state) {
        pre_state_ = state_;
        state_     = new_state;
    }

    void ResetTracking() { tracking_car_id_ = enumeration::CarIDFlag::None; }

    world_exe::enumeration::CarIDFlag tracking_car_id_ { enumeration::CarIDFlag::None };
    TrackState state_     = TrackState::Lost;
    TrackState pre_state_ = TrackState::Lost;
    CarIDFlag invincible_armors_ { CarIDFlag::None };

    int detect_count_                      = 0;
    int temp_lost_count_                   = 0;
    int max_temp_lost_count_               = 15;
    const int min_detect_count_            = 5;
    const int outpost_max_temp_lost_count_ = 75;
    const int normal_max_temp_lost_count_  = max_temp_lost_count_;
    const int max_switch_count_            = 200;

    predictor::TimeStamp last_timestamp_;

    std::unique_ptr<decider::Decider> decider_;
};

}