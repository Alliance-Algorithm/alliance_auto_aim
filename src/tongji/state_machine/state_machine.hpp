#pragma once

#include <ctime>
#include <memory>

#include "enum/car_id.hpp"
#include "interfaces/car_state.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::state_machine {
class StateMachineImpl;
class StateMachine final : public interfaces::ICarState {
public:
    StateMachine();
    ~StateMachine();

    const enumeration ::CarIDFlag& GetAllowdToFires() const override;
    StateMachine(std::shared_ptr<world_exe::interfaces::ITargetPredictor> live_target_manager);

    StateMachine(const StateMachine&)                = delete;
    StateMachine& operator=(const StateMachine&)     = delete;
    StateMachine(StateMachine&&) noexcept            = default;
    StateMachine& operator=(StateMachine&&) noexcept = default;

private:
    std::unique_ptr<StateMachineImpl> pimpl_;
};
}