#pragma once

#include "car_predictor_ekf.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/predictor.hpp"

namespace world_exe::predictor {
class CarPredictor : public interfaces::IPredictor {
public:
    CarPredictor();
    ~CarPredictor();
    CarPredictor(const enumeration::CarIDFlag& id, const CarPredictEkf& ekf,
        const interfaces::ITimeStamped& create_time_stamp);

    void SetId(const enumeration::CarIDFlag& id);
    void SetEkf(const CarPredictEkf& ekf);
    void SetTimeStamp(const interfaces::ITimeStamped& time_stamp);

    const enumeration::ArmorIdFlag& GetId() const override;
    const interfaces::IArmorInGimbalControl& Predictor(const std::time_t& time_stamp) override;
private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}