#pragma once

#include <memory>

#include "enum/car_id.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::tongji::predictor {
class CarPredictorManager final : public interfaces::IPredictor {
public:
    CarPredictorManager();
    const enumeration ::ArmorIdFlag& GetId() const override;

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predictor(
        const std ::time_t& time_stamp) const override;

    void SetPredictorBySingleID(const enumeration::CarIDFlag& id,
        Eigen::Vector3d armor_xyz_in_world, Eigen::Vector3d armor_ypr_in_world, time_t time_stamp);

    void SetPredictorBySingleID(const enumeration::CarIDFlag& id, time_t time_stamp);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}