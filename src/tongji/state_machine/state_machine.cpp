#include "state_machine.hpp"

#include <memory>

#include "enum/car_id.hpp"
#include "interfaces/target_predictor.hpp"
#include "tongji/predictor/live_target_manager/live_target_manager.hpp"

namespace world_exe::tongji::state_machine {

class StateMachine::Impl {
public:
    Impl(std::shared_ptr<world_exe::interfaces::ITargetPredictor> live_target_manager)
        : live_target_manager_(live_target_manager)
        , target_ids_(enumeration::CarIDFlag::None) { }

    const enumeration::CarIDFlag& GetAllowdToFires() const {
        auto live_target_manager =
            std::dynamic_pointer_cast<predictor::LiveTargetManager>(live_target_manager_);
        target_ids_ = live_target_manager->GetAllowedTargetID();

        return target_ids_;
    }

private:
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;
    mutable enumeration::CarIDFlag target_ids_;
};

StateMachine::StateMachine(std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
    : pimpl_(std::make_unique<Impl>(live_target_manager)) { }
StateMachine::~StateMachine() = default;

const enumeration::CarIDFlag& StateMachine::GetAllowdToFires() const {
    return pimpl_->GetAllowdToFires();
}
}