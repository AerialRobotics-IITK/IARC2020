#include <agent_state_machine/behaviours/termination.hpp>

namespace ariitk::agent_state_machine {

void Termination::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr) {
    mav_state_ = state_ptr;
}

void Termination::execute(const Event& evt) {
    BHV_INFO("Switching to Land...");
    bool result = mav_state_->switchMode("AUTO.LAND");
}

}  // namespace ariitk::agent_state_machine
