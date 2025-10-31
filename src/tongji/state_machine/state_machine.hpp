#pragma once

#include <ctime>
#include <memory>

#include "enum/car_id.hpp"
#include "interfaces/armor_in_image.hpp"
#include "interfaces/car_state.hpp"

namespace world_exe::tongji::predictor {
class LiveTargetManager;
}

namespace world_exe::tongji::state_machine {
class StateMachine final : public interfaces::ICarState {
public:
    StateMachine();
    ~StateMachine();

    const enumeration ::CarIDFlag& GetAllowdToFires() const override;

    void Update(std::shared_ptr<interfaces::IArmorInImage> armors_in_image);

    StateMachine(const StateMachine&)                = delete;
    StateMachine& operator=(const StateMachine&)     = delete;
    StateMachine(StateMachine&&) noexcept            = default;
    StateMachine& operator=(StateMachine&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}
