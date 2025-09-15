#include "car_predictor_boss.hpp"

#include <cstdint>
#include <ctime>
#include <memory>

#include "enum/car_id.hpp"
#include "enum/enum_tools.hpp"
#include "tongji/predictor/car_predictor_manager.hpp"

namespace world_exe::tongji::predictor {
class CarPredictorBoss::Impl {
public:
    Impl(std::shared_ptr<CarPredictorManager> manager)
        : manager_(manager) {
        if (!manager_) {
            throw std::invalid_argument("CarPredictorManager must not be null");
        }
    }

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
        for (int i = 0; i < 8; ++i) {
            auto car_id = static_cast<enumeration::CarIDFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);

            if (enumeration::IsFlagContains(id, car_id)) {
                manager_->SetPredictorBySingleID(car_id, time_stamp);
            }
        }
        return manager_->Predictor(time_stamp);
    }

    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const {
        return manager_;
    }

private:
    std::shared_ptr<CarPredictorManager> manager_;
};

std ::shared_ptr<interfaces ::IArmorInGimbalControl> CarPredictorBoss::Predict(
    const enumeration ::ArmorIdFlag& id, const std ::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}

std ::shared_ptr<interfaces::IPredictor> CarPredictorBoss::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

CarPredictorBoss::CarPredictorBoss(std::shared_ptr<CarPredictorManager> manager)
    : pimpl_(std::make_unique<Impl>(manager)) { }

}