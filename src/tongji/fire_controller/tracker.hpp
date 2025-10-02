#pragma once

#include "../predictor/target_snapshot.hpp"
#include "../predictor/target_snapshot_manager.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/car_state.hpp"
#include "tongji/identifier/identified_armor.hpp"
#include <memory>
#include <opencv2/core/types.hpp>

class DefaultTracker final {
    using TargetSnapshotManager = world_exe::tongji::predictor::TargetSnapshotManager;
    using TargetSnapshot        = world_exe::tongji::predictor::TargetSnapshot;
    using ArmorInImage          = world_exe::tongji::identifier::IdentifiedArmor;
    using EnemiesState          = world_exe::interfaces::ICarState;

public:
    DefaultTracker() = default;

    ~DefaultTracker() = default;

    auto CalculateTarget(                                               //
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

        auto iterator    = sq_armor_list->begin();
        auto filter_flag = snapshot_manager_->GetId();

        while (!world_exe::enumeration::IsFlagContains(filter_flag, iterator->armor.id)) {
            ++iterator;
            if (iterator == sq_armor_list->end()) return nullptr;
        }
        return snapshot_manager_->GetSingleSnapshot(iterator->armor.id);
    }

private:
};
