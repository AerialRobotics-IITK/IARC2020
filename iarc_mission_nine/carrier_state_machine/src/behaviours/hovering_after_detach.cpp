#include <carrier_state_machine/behaviours/hovering_after_detach.hpp>

namespace ariitk::carrier_state_machine {

void HoveringAfterDetach::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
    mav_state_ = state_ptr;
}

void HoveringAfterDetach::execute(const Event& evt) {
    bool result = mav_state_->switchMode("AUTO.LOITER");
}

}  // namespace ariitk::carrier_state_machine
