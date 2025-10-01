#pragma once

#include "../predictor/target_snapshot.hpp"
#include "../predictor/target_snapshot_manager.hpp"

class DefaultTracker {
    using TargetSnapshotManager = world_exe::tongji::predictor::TargetSnapshotManager;
    using TargetSnapshot        = world_exe::tongji::predictor::TargetSnapshot;

public:
    DefaultTracker(const std::shared_ptr<TargetSnapshotManager>& snapshot_manager)
        : snapshot_manager_(snapshot_manager) {};

    ~DefaultTracker() = default;

    auto CalculateTarget() noexcept -> TargetSnapshot {
        // tongji shooter::tracker impl here
    }

private:
    const std::shared_ptr<TargetSnapshotManager> snapshot_manager_;
};