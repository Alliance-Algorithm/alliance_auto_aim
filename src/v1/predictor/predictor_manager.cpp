#include "predictor_manager.hpp"
#include "car/car_predictor.hpp"
#include "car/car_predictor_ekf.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/car_state.hpp"
#include "predict_armor_in_gimbal_control.hpp"
#include "predict_time_stamp.hpp"
#include "util/index.hpp"

namespace world_exe::v1::predictor {

class PredictorManager::Impl {
public:
    inline void Update(const interfaces::IPreDictorUpdatePackage& data) {
        const auto time_stamp = data.GetTimeStamped().GetTimeStamp();
        const auto dt         = (time_stamp - last_update_time_stamp_.GetTimeStamp()) / 1.e9;

        const auto transform          = data.GetTransform();
        const auto rotation_transform = Eigen::Quaterniond { transform.linear() };

        for (int i = 0; i < 8; i++) {
            const auto& armors =
                data.GetArmors().GetArmors(static_cast<enumeration::ArmorIdFlag>(0b00000001 << i));
            if (armors.empty()) continue;
            CarPredictEkf::ZVec input;
            if (armors.size() == 1) {
                const auto tmp_armor = data::ArmorGimbalControlSpacing { armors[0].id,
                    transform * armors[0].position, rotation_transform * armors[0].orientation };
                input << util::math::get_yaw_from_quaternion(tmp_armor.orientation),
                    std::atan(tmp_armor.position.y() / tmp_armor.position.x()),
                    -std::atan(tmp_armor.position.z() / tmp_armor.position.x()),
                    tmp_armor.position.norm();
                predictors_[i].Update(input, {}, dt);
            } else if (armors.size() == 2) {
                const auto armor0_yaw = util::math::get_yaw_from_quaternion(armors[0].orientation);
                const auto armor1_yaw = util::math::get_yaw_from_quaternion(armors[1].orientation);

                const auto tmp_armor0 = data::ArmorGimbalControlSpacing { armors[0].id,
                    transform * armors[0].position, rotation_transform * armors[0].orientation };
                const auto tmp_armor1 = data::ArmorGimbalControlSpacing { armors[1].id,
                    transform * armors[1].position, rotation_transform * armors[1].orientation };

                if (std::abs(armor0_yaw) > std::abs(armor1_yaw)) {
                    input << util::math::get_yaw_from_quaternion(tmp_armor0.orientation),
                        std::atan(tmp_armor0.position.y() / tmp_armor0.position.x()),
                        -std::atan(tmp_armor0.position.z() / tmp_armor0.position.x()),
                        tmp_armor0.position.norm();
                    predictors_[i].Update(input, {}, dt);
                    predictors_[i].set_second_armor();
                    input << util::math::get_yaw_from_quaternion(tmp_armor1.orientation),
                        std::atan(tmp_armor1.position.y() / tmp_armor1.position.x()),
                        -std::atan(tmp_armor1.position.z() / tmp_armor1.position.x()),
                        tmp_armor1.position.norm();
                    predictors_[i].Update(input, {}, 0.);
                } else {
                    input << util::math::get_yaw_from_quaternion(tmp_armor1.orientation),
                        std::atan(tmp_armor1.position.y() / tmp_armor1.position.x()),
                        -std::atan(tmp_armor1.position.z() / tmp_armor1.position.x()),
                        tmp_armor1.position.norm();
                    predictors_[i].Update(input, {}, dt);
                    predictors_[i].set_second_armor();
                    input << util::math::get_yaw_from_quaternion(tmp_armor0.orientation),
                        std::atan(tmp_armor0.position.y() / tmp_armor0.position.x()),
                        -std::atan(tmp_armor0.position.z() / tmp_armor0.position.x()),
                        tmp_armor0.position.norm();
                    predictors_[i].Update(input, {}, 0.);
                }
            }
        }

        last_update_time_stamp_.SetTimeStamp(time_stamp);
    }

    inline const world_exe::interfaces::IArmorInGimbalControl& Predict(
        const world_exe::enumeration::ArmorIdFlag& id, const std::time_t& time_stamp) {
        const auto dt = (time_stamp - last_update_time_stamp_.GetTimeStamp()) / 1.e9;

        uint32_t id_index = static_cast<uint32_t>(enumeration::ArmorIdFlag::Hero);
        std::array<std::vector<data::ArmorGimbalControlSpacing>, 8> armors;
        for (int i = 0; i < 8; i++)
            if (enumeration::IsFlagContains(
                    id, static_cast<enumeration::ArmorIdFlag>(0b00000001 << i)))
                armors[i] = predictors_[i].get_predict_output_armor(
                    static_cast<enumeration::CarIDFlag>(0b00000001 << i), dt);

        predictted_armors_.Set(armors, { time_stamp });

        return predictted_armors_;
    };

    const interfaces::IPredictor& GetPredictor(const enumeration::ArmorIdFlag& id) {
        predictor.SetId(id);
        predictor.SetEkf(predictors_[util::enumeration::GetIndex(id)]);
        predictor.SetTimeStamp(last_update_time_stamp_);
        return predictor;
    }

private:
    PredictTimeStamp last_update_time_stamp_ { 0 };
    std::array<CarPredictEkf, 8> predictors_;

    Eigen::Affine3d transform_from_camera_to_gimbal_;
    PredictArmorInGimbalControl predictted_armors_;
    CarPredictor predictor;
};

PredictorManager::PredictorManager()
    : pimpl_(std::make_unique<Impl>()) { }

PredictorManager::~PredictorManager() = default;

const world_exe::interfaces::IArmorInGimbalControl& PredictorManager::Predict(
    const world_exe::enumeration::ArmorIdFlag& id, const std::time_t& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
};

void PredictorManager::Update(const interfaces::IPreDictorUpdatePackage& data) {
    return pimpl_->Update(data);
};

const interfaces::IPredictor& PredictorManager::GetPredictor(
    const enumeration::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
};

}