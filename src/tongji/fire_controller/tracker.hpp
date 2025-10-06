#pragma once

#include <memory>

#include <opencv2/core/types.hpp>

#include "../predictor/target_snapshot.hpp"
#include "../predictor/target_snapshot_manager.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/car_state.hpp"
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

class DefaultTracker final {
    using TargetSnapshotManager = world_exe::tongji::predictor::TargetSnapshotManager;
    using TargetSnapshot        = world_exe::tongji::predictor::TargetSnapshot;
    using ArmorInImage          = world_exe::tongji::identifier::IdentifiedArmor;
    using EnemiesState          = world_exe::interfaces::ICarState;

public:
    DefaultTracker()
        : last_timestamp_(0) { }

    ~DefaultTracker() = default;

    auto SelectTrackingTarget(                                          //
        ArmorInImage& armors,                                           //
        const std::shared_ptr<TargetSnapshotManager>& snapshot_manager_ //
        ) noexcept -> std::unique_ptr<TargetSnapshot> {

        const auto& sq_armor_list = armors.get_sq_armor();

        sq_armor_list->sort([](const world_exe::tongji::identifier::SPArmor& a,
                                const world_exe::tongji::identifier::SPArmor& b) {
            cv::Point2f img_center(1440.0 / 2, 1080.0 / 2); // TODO
            auto distance_1 = cv::norm(a.center - img_center);
            auto distance_2 = cv::norm(b.center - img_center);
            return distance_1 < distance_2;
        });
        sq_armor_list->sort([](const auto& a, const auto& b) { return a.priority < b.priority; });

        // auto iterator    = sq_armor_list->begin();
        auto filter_flag = snapshot_manager_->GetId();
        for (const auto& armor : *sq_armor_list) {
            if (!enumeration::IsFlagContains(filter_flag, armor.armor.id)) continue;

            auto snapshot = snapshot_manager_->GetSingleSnapshot(armor.armor.id);
            if (!snapshot) continue;

            if (state_ == TrackState::Tracking && snapshot->GetPriority() < current_priority_) {
                SetState(TrackState::Switching);
                temp_lost_count_ = 0;
            }

            auto now = predictor::TimeStamp(std::time(nullptr));
            if (state_ != TrackState::Lost && now.SecondsSince(last_timestamp_) > 0.1) {
                SetState(TrackState::Lost);
                ResetTracking();
                return nullptr;
            }

            last_timestamp_   = now;
            tracking_car_id_  = snapshot->GetID();
            current_priority_ = snapshot->GetPriority();
            return snapshot;
        }
        ResetTracking();
        return nullptr;
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

    void ResetTracking() {
        tracking_car_id_  = enumeration::CarIDFlag::None;
        current_priority_ = 100;
    }

    int current_priority_ = 100;
    world_exe::enumeration::CarIDFlag tracking_car_id_ { enumeration::CarIDFlag::None };
    TrackState state_     = TrackState::Lost;
    TrackState pre_state_ = TrackState::Lost;

    int detect_count_                      = 0;
    int temp_lost_count_                   = 0;
    int max_temp_lost_count_               = 15;
    const int min_detect_count_            = 5;
    const int outpost_max_temp_lost_count_ = 75;
    const int normal_max_temp_lost_count_  = max_temp_lost_count_;
    const int max_switch_count_            = 200;

    predictor::TimeStamp last_timestamp_;
};

}