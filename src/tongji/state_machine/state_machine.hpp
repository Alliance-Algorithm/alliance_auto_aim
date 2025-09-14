#pragma once

#include <chrono>

#include "car_state_manager.hpp"
#include "enum/car_id.hpp"
#include "interfaces/car_state.hpp"

namespace world_exe::tongji::state_machine {
class StateMachine final : public interfaces::ICarState {
public:
    StateMachine(int switch_threshold = 5);

    const enumeration::CarIDFlag& GetAllowdToFires() const override;

    const interfaces::ICarState& Update(
        const enumeration::CarIDFlag& car_detected, std::chrono::steady_clock::time_point now);

private:
    enumeration::CarIDFlag tracing_state_ = enumeration::CarIDFlag::None;
    std::array<car_state::CarStateManager, 8> car_states_;
    int switch_threshold_;
};
}