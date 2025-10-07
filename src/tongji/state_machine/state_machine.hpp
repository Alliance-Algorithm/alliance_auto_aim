#pragma once

#include <ctime>
#include <memory>

#include "enum/car_id.hpp"
#include "interfaces/car_state.hpp"
#include "tongji/state_machine/car_state_manager.hpp"

namespace world_exe::tongji::state_machine {
class StateMachine final : public interfaces::ICarState {
public:
    StateMachine();
    ~StateMachine();

    const enumeration ::CarIDFlag& GetAllowdToFires() const override;
    const car_state::CarStateManager& GetState(enumeration::CarIDFlag single_id) const;
    bool IsAllowedToFire(enumeration::CarIDFlag id) const;
    const interfaces::ICarState& Update(
        const enumeration::CarIDFlag& car_detected, const std::time_t& now);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}