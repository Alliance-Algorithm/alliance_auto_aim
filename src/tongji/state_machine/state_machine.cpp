#include "state_machine.hpp"

#include <array>

#include "enum/car_id.hpp"
#include "tongji/state_machine/car_state_manager.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::state_machine {
using TimeStamp = car_state::TimeStamp;

class StateMachine::Impl {
public:
    Impl()
        : car_states_ { } {
        std::generate(
            car_states_.begin(), car_states_.end(), []() { return car_state::CarStateManager(); });
    }

    const enumeration::CarIDFlag& GetAllowdToFires() const { return tracing_state_; }

    bool IsAllowedToFire(enumeration::CarIDFlag id) const { return tracing_state_ == id; }

    const car_state::CarStateManager& GetState(enumeration::CarIDFlag single_id) const {
        return car_states_[util::enumeration::GetIndex(single_id)];
    }
    void Update(const enumeration::CarIDFlag& car_detected, const std::time_t& now) {
        tracing_state_    = enumeration::CarIDFlag::None;
        int best_priority = std::numeric_limits<int>::max();

        for (int i = 0; i < static_cast<int>(enumeration::CarIDFlag::Count); ++i) {
            auto id       = static_cast<enumeration::CarIDFlag>(1 << i);
            bool detected = (static_cast<int>(car_detected) >> i) & 0x01;

            auto& state = car_states_[i];
            state.Update(detected, now);

            if (!state.IsConverged()) {
                state.Reset();
                continue;
            }

            if (state.IsLost(now)) {
                state.Reset();
                continue;
            }

            int p = state.GetPriority();
            if (p < best_priority) {
                best_priority  = p;
                tracing_state_ = id;
            }
        }
    }

private:
    enumeration::CarIDFlag tracing_state_ = enumeration::CarIDFlag::None;
    std::array<car_state::CarStateManager, 8> car_states_;
};

StateMachine::StateMachine()
    : pimpl_(std::make_unique<Impl>()) { }

StateMachine::~StateMachine() = default;

const enumeration::CarIDFlag& StateMachine::GetAllowdToFires() const {
    return pimpl_->GetAllowdToFires();
}

bool StateMachine::IsAllowedToFire(enumeration::CarIDFlag id) const {
    return pimpl_->IsAllowedToFire(id);
}

const car_state::CarStateManager& StateMachine::GetState(enumeration::CarIDFlag single_id) const {
    return pimpl_->GetState(single_id);
}

const interfaces::ICarState& StateMachine::Update(
    const enumeration::CarIDFlag& car_detected, const std::time_t& now) {
    pimpl_->Update(car_detected, now);
    return *this;
}

}
