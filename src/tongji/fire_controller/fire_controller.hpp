#pragma once

#include <memory>

#include "interfaces/fire_controller.hpp"
#include "tongji/predictor/target_snapshot_manager.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController();

    const data ::FireControl CalculateTarget(const std ::time_t& time_duration) const override;

    const enumeration ::CarIDFlag GetAttackCarId() const override;

    void SetSnapshotManager(std::shared_ptr<predictor::TargetSnapshotManager> manager);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}