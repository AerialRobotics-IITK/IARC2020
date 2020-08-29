#include <carrier_state_machine/behaviours/hovering.hpp>

namespace ariitk::carrier_state_machine {

void Hovering::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
    mav_state_ = state_ptr;
}

void Hovering::execute(const Event& evt) {
    bool result = mav_state_->switchMode("AUTO.LOITER");
}

}  // namespace ariitk::carrier_state_machine
