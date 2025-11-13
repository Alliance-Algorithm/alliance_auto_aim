#include "state_machine.hpp"

#include <memory>

#include "../identifier/tracker.hpp"
#include "data/time_stamped.hpp"
#include "enum/car_id.hpp"

namespace world_exe::tongji::state_machine {

class StateMachine::Impl {
public:
    Impl()
        : tracker_(std::make_unique<identifier::Tracker>())
        , target_id_(enumeration::CarIDFlag::None) { }

    const enumeration::CarIDFlag& GetAllowdToFires() const { return target_id_; }

    void Update(std::shared_ptr<interfaces::IArmorInImage> const& armors_in_image,
        const enumeration::CarIDFlag& invincible_armors, data::TimeStamp const& time_stamp) {
        target_id_ = enumeration::CarIDFlag::None;
        target_id_ =
            tracker_->SelectTrackingTargetID(armors_in_image, invincible_armors, time_stamp);
    }

    void SetLostState() {
        target_id_ = enumeration::CarIDFlag::None;
        tracker_->SetLostState();
    }

private:
    std::unique_ptr<identifier::Tracker> tracker_;
    enumeration::CarIDFlag target_id_;
};

StateMachine::StateMachine()
    : pimpl_(std::make_unique<Impl>()) { }
StateMachine::~StateMachine() { };

const enumeration::CarIDFlag& StateMachine::GetAllowdToFires() const {
    return pimpl_->GetAllowdToFires();
}

void StateMachine::Update(std::shared_ptr<interfaces::IArmorInImage> const& armors_in_image,
    const enumeration::CarIDFlag& invincible_armors, data::TimeStamp const& time_stamp) {
    return pimpl_->Update(armors_in_image, invincible_armors, time_stamp);
}

void StateMachine::SetLostState() { return pimpl_->SetLostState(); }
}
