#pragma once

#include "interfaces/target_predictor.hpp"

namespace world_exe::predictor {
class PredictorManager : public world_exe::interfaces::ITargetPredictor {
public:
    PredictorManager();
    ~PredictorManager();

    void Update(const interfaces::IPreDictorUpdatePackage& data) override;

    virtual const interfaces::IArmorInGimbalControl& Predict(
        const enumeration::ArmorIdFlag& id, const std::time_t& time_stamp) override;

    const interfaces::IPredictor& GetPredictor(const enumeration::ArmorIdFlag& id) const override;
private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}