#include <agent_state_machine/behaviours/termination.hpp>

namespace ariitk::agent_state_machine {

void Termination::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr) {
    mav_state_ = state_ptr;
}

void Termination::execute(const Event evt) {
    BHV_INFO("Landing...");
    geometry_msgs::Pose land_setpt = mav_state_->getPose();
    land_setpt.position.z = 0.2;  // TODO: Set a land height param
    mav_state_->goToLocation(land_setpt);
}

}  // namespace ariitk::agent_state_machine