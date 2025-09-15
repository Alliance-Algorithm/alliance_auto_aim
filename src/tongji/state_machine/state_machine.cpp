#include "state_machine.hpp"

#include "enum/car_id.hpp"

namespace world_exe::tongji::state_machine {

StateMachine::StateMachine(int switch_threshold)
    : switch_threshold_(switch_threshold) {
    for (auto& manager : car_states_) {
        manager = car_state::CarStateManager(switch_threshold_);
    }
}

const enumeration::CarIDFlag& StateMachine::GetAllowdToFires() const { return tracing_state_; }

const interfaces::ICarState& StateMachine::Update(
    const enumeration::CarIDFlag& car_detected, std::time_t now) {
    tracing_state_ = enumeration::CarIDFlag::None;

    for (int i = 0; i < static_cast<int>(enumeration::CarIDFlag::Count); ++i) {
        auto id       = static_cast<enumeration::CarIDFlag>(1 << i);
        bool detected = (static_cast<int>(car_detected) >> i) & 0x01;

        car_states_[i].Update(detected, now);

        if (detected) {
            car_states_[i].IncrementUpdateCount();
        }

        if (car_states_[i].IsLocked() && car_states_[i].IsConverged()
            && !car_states_[i].IsDiverged()) {
            tracing_state_ =
                static_cast<enumeration::CarIDFlag>(static_cast<int>(tracing_state_) | (1 << i));
        }
    }

    return *this;
}

}
