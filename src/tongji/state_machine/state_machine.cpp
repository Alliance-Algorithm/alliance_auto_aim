#include "state_machine.hpp"

#include <memory>

#include "enum/car_id.hpp"
#include "tongji/predictor/live_target_manager/live_target_manager.hpp"

namespace world_exe::tongji::state_machine {

class StateMachine::Impl {
public:
    Impl(std::shared_ptr<predictor::LiveTargetManager> live_target_manager)
        : live_target_manager_(live_target_manager)
        , target_ids_(enumeration::CarIDFlag::None) { }

    /// but why?
    const enumeration::CarIDFlag& GetAllowdToFires() const {
        target_ids_ = live_target_manager_->GetAllowedTargetID();
        return target_ids_;
    }

private:
    std::shared_ptr<predictor::LiveTargetManager> live_target_manager_;
    mutable enumeration::CarIDFlag target_ids_;
};

StateMachine::StateMachine(std::shared_ptr<predictor::LiveTargetManager> live_target_manager)
    : pimpl_(std::make_unique<Impl>(live_target_manager)) { }

StateMachine::~StateMachine() {};
StateMachine::StateMachine() {};

const enumeration::CarIDFlag& StateMachine::GetAllowdToFires() const {
    return pimpl_->GetAllowdToFires();
}
}