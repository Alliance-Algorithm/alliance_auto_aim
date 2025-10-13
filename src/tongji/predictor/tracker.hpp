#pragma once

#include <memory>
#include <opencv2/core/types.hpp>
#include <utility>
#include <vector>

#include "enum/armor_id.hpp"
#include "tongji/identifier/armor_filter.hpp"
#include "tongji/identifier/identified_armor.hpp"
#include "tongji/predictor/decider.hpp"
#include "tongji/predictor/target_snapshot.hpp"
#include "tongji/predictor/target_snapshot_manager.hpp"
#include "tongji/predictor/time_stamp.hpp"

namespace world_exe::tongji::predictor {

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

public:
    Tracker()
        : armor_filter_(std::make_unique<identifier::ArmorFilter>())
        , decider_(std::make_unique<Decider>())
        , last_track_timestamp_(std::time(nullptr)) { }

    ~Tracker() = default;

    auto SelectTrackingTargetID(const std::shared_ptr<interfaces::IArmorInImage>& armors_in_image,
        const std::time_t& now) noexcept -> enumeration::ArmorIdFlag const {
        CheckCameraOffline(now);
        last_track_timestamp_.SetTimeStamp(now);

        auto filtered_ids = enumeration::ArmorIdFlag::None;
        auto detected_ids = enumeration::ArmorIdFlag::None;
        std::vector<data::ArmorImageSpacing> filtered_armors;
        for (uint32_t i = 0; i < static_cast<int>(enumeration::ArmorIdFlag::Count); ++i) {
            auto id = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(enumeration::ArmorIdFlag::Hero) << i);

            if (armors_in_image->GetArmors(id).empty()) continue;

            // 图像中出现的装甲板
            auto armors  = armors_in_image->GetArmors(id);
            detected_ids = static_cast<enumeration::ArmorIdFlag>(
                static_cast<uint32_t>(detected_ids) | static_cast<uint32_t>(id));

            // 对从图像识别到的装甲板进行过滤
            filtered_armors = std::move(armor_filter_->FilterArmor(std::move(armors)));
            if (!filtered_armors.empty()) {
                filtered_ids =
                    static_cast<enumeration::ArmorIdFlag>(static_cast<uint32_t>(filtered_ids)
                        | static_cast<uint32_t>(filtered_armors.at(0).id));
            }
        }

        UpdateState(!(detected_ids == enumeration::ArmorIdFlag::None));

        tracking_car_id_ = decider_->GetBestArmor(filtered_armors);
        return tracking_car_id_;
    }

    void UpdateState(bool found) {
        switch (state_) {
        case TrackState::Lost: {
            if (found) {
                SetState(TrackState::Detecting);
                detect_count_ = 1;
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
    void CheckCameraOffline(const std::time_t& now) {
        // TODO:If the underlying timestamp is std::time_t, then this if branch will never be
        // entered
        if (state_ != TrackState::Lost
            && static_cast<double>(now - last_track_timestamp_.GetTimeStamp()) < 0.1)
            SetState(TrackState::Lost);
    }

    void SetState(TrackState new_state) {
        pre_state_ = state_;
        state_     = new_state;
    }

    void ResetTracking() { tracking_car_id_ = enumeration::CarIDFlag::None; }

    world_exe::enumeration::CarIDFlag tracking_car_id_ { enumeration::CarIDFlag::None };
    TrackState state_     = TrackState::Lost;
    TrackState pre_state_ = TrackState::Lost;

    std::unique_ptr<identifier::ArmorFilter> armor_filter_;
    std::unique_ptr<Decider> decider_;

    int detect_count_                      = 0;
    int temp_lost_count_                   = 0;
    int max_temp_lost_count_               = 15;
    const int min_detect_count_            = 5;
    const int outpost_max_temp_lost_count_ = 75;
    const int normal_max_temp_lost_count_  = max_temp_lost_count_;
    const int max_switch_count_            = 200;

    predictor::TimeStamp last_track_timestamp_;
};

}