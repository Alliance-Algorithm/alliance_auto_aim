#include "car_predictor.hpp"
#include "../predict_armor_in_gimbal_control.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/time_stamped.hpp"
#include "v1/predictor/predict_time_stamp.hpp"
#include <memory>

namespace world_exe::predictor {
class CarPredictor::Impl {
public:
    Impl(const enumeration::CarIDFlag& id, const CarPredictEkf& ekf,
        const interfaces::ITimeStamped& create_time_stamp)
        : id_(id)
        , ekf_(ekf) {
        create_time_stamp_.SetTimeStamp(create_time_stamp.GetTimeStamp());
    }

    const enumeration::ArmorIdFlag& GetId() const { return id_; }

    const interfaces::IArmorInGimbalControl& Predictor(const std::time_t& time_stamp) {
        target_armors_.SetWithSingleId(ekf_.get_predict_output_armor(id_,
                                           (time_stamp - create_time_stamp_.GetTimeStamp()) / 1.e9),
            time_stamp);
        return target_armors_;
    }

    inline void SetId(const enumeration::CarIDFlag& id) { id_ = id; }

    inline void SetEkf(const CarPredictEkf& ekf) { ekf_ = ekf; }

    inline void SetTimeStamp(const interfaces::ITimeStamped& time_stamp) {
        create_time_stamp_.SetTimeStamp(time_stamp.GetTimeStamp());
    }

private:
    enumeration::CarIDFlag id_;
    CarPredictEkf ekf_;
    PredictTimeStamp create_time_stamp_;

    PredictArmorInGimbalControl target_armors_;
};

CarPredictor::CarPredictor()
    : pimpl_(std::make_unique<Impl>(
          enumeration::CarIDFlag::None, CarPredictEkf {}, PredictTimeStamp {})) { }

CarPredictor::CarPredictor(const enumeration::CarIDFlag& id, const CarPredictEkf& ekf,
    const interfaces::ITimeStamped& create_time_stamp)
    : pimpl_(std::make_unique<Impl>(id, ekf, create_time_stamp)) { }

CarPredictor::~CarPredictor() = default;

const enumeration::ArmorIdFlag& CarPredictor::GetId() const { return pimpl_->GetId(); }

const interfaces::IArmorInGimbalControl& CarPredictor::Predictor(const std::time_t& time_stamp) {
    return pimpl_->Predictor(time_stamp);
}

inline void CarPredictor::SetId(const enumeration::CarIDFlag& id) { return pimpl_->SetId(id); }

inline void CarPredictor::SetEkf(const CarPredictEkf& ekf) { return pimpl_->SetEkf(ekf); }

inline void CarPredictor::SetTimeStamp(const interfaces::ITimeStamped& time_stamp) {
    return pimpl_->SetTimeStamp(time_stamp);
};
}