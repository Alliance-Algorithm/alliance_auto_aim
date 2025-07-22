
#include "./fire_controller.hpp"
#include "Eigen/src/Geometry/Quaternion.h"
#include "enum/armor_id.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/predictor.hpp"
#include "trajectory.hpp"
#include <memory>
#include <stdexcept>

class world_exe::v1::fire_control::TracingFireControl::TracingFireSelectImpl {
public:
    TracingFireSelectImpl(const interfaces::IArmorInCamera& armors)
        : armors_(armors) { }
    const enumeration::CarIDFlag GetAttackCarId() const {
        double max_dot = -2;

        enumeration::CarIDFlag ret = enumeration::ArmorIdFlag::None;
        for (auto i = enumeration::ArmorIdFlag::Hero; i < enumeration::ArmorIdFlag::Count;
            i       = static_cast<decltype(i)>((int)i << 1)) {
            if (world_exe::enumeration::IsFlagContains(tracing_, i)) {
                const auto& armors_vector = armors_.GetArmors(i);
                for (const auto& a : armors_vector) {
                    auto dot = a.position.normalized().dot(Eigen::Vector3d::UnitX());
                    if (dot > max_dot) {
                        ret     = i;
                        max_dot = dot;
                    }
                }
            }
        }
        return ret;
    }
    void SetTargetCarID(const world_exe::enumeration::CarIDFlag& id) { tracing_ = id; }

private:
    world_exe::enumeration::CarIDFlag tracing_ = enumeration::CarIDFlag::None;
    const interfaces::IArmorInCamera& armors_;
};
class world_exe::v1::fire_control::TracingFireControl::TracingFireCalculateImpl {

public:
    TracingFireCalculateImpl(interfaces::IPredictor& predictor)
        : predictor_(predictor) { }
    const world_exe::data::FireControl CalculateTarget(const std::time_t& time_duration,
        const time_t& time_predict_point, const time_t& control_delay_, double velocity_begin,
        double gravity) {
        throw std::runtime_error("Wait for rewirte here at FireController.cpp");

        time_t fly_time        = 0;
        const auto& pre1       = predictor_.Predictor(fly_time + time_duration + control_delay_);
        const auto& pre2       = pre1.GetArmors(predictor_.GetId());
        double min_angular_dis = 1e9;
        int index = -1, index_ = 0;
        for (const auto vec : pre2) {
            const auto angular_dis =
                vec.orientation.angularDistance(Eigen::Quaterniond::Identity());

            if (angular_dis < min_angular_dis) {
                index           = index_;
                min_angular_dis = angular_dis;
            }

            index_++;
        }
        if (index == -1) return no_allow_;

        for (int i = 5; i-- > 0;) {
            const auto& armors_in_gimbal_control =
                predictor_.Predictor(fly_time + time_duration + control_delay_);
            const auto& armors = armors_in_gimbal_control.GetArmors(predictor_.GetId());
            const auto& [fly_time, dir] =
                trajectory_solver::gravity_only(armors[index].position, velocity_begin, gravity);
        }

        return { .time_stamp = fly_time + time_duration + control_delay_, .fire_allowance = true };
    }

private:
    interfaces::IPredictor& predictor_;
};

const world_exe::data::FireControl //
world_exe::v1::fire_control::TracingFireControl::CalculateTarget(
    const std::time_t& time_duration) const {
    if (p_calc_impl_ != nullptr) [[unlikely]]
        return no_allow_;
    return p_calc_impl_->CalculateTarget(
        time_duration, time_stamp_, control_delay_, velocity_begin_, gravity_);
}

const world_exe::enumeration::CarIDFlag
world_exe::v1::fire_control::TracingFireControl::GetAttackCarId() const {
    if (p_select_impl_ != nullptr) [[unlikely]]
        return p_select_impl_->GetAttackCarId();
    return world_exe::enumeration::CarIDFlag::None;
}
world_exe::v1::fire_control::TracingFireControl::TracingFireControl(
    double t, double velocity_begin, double gravity)
    : velocity_begin_(velocity_begin)
    , gravity_(gravity) {
    control_delay_ = static_cast<time_t>(t * 1E9);
}

void world_exe::v1::fire_control::TracingFireControl::SetArmorsInGimbalControl(
    const interfaces::IArmorInCamera& armors) {
    p_select_impl_ = std::make_unique<TracingFireSelectImpl>(armors);
}

void world_exe::v1::fire_control::TracingFireControl::SetPredictor(
    interfaces::IPredictor& predictor) {
    p_calc_impl_ = std::make_unique<TracingFireCalculateImpl>(predictor);
}

void world_exe::v1::fire_control::TracingFireControl::SetTargetCarID(
    const world_exe::enumeration::CarIDFlag& tracing_id) {
    if (p_select_impl_ != nullptr) [[unlikely]]
        p_select_impl_->SetTargetCarID(tracing_id);
}

void world_exe::v1::fire_control::TracingFireControl::SetTimeStamp(const time_t& time) {
    time_stamp_ = time;
}