#include <agent_state_machine/behaviours/hovering.hpp>

namespace ariitk::agent_state_machine {

void Hovering::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr) {
    mav_state_ = state_ptr;
}

void Hovering::execute(const Event& evt) {
    bool result = mav_state_->switchMode("AUTO.HOLD");
}

}  // namespace ariitk::agent_state_machine
