#include <agent_state_machine/behaviours/initialization.hpp>

namespace ariitk::agent_state_machine {

void Initialization::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr) {
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("call_rate", call_rate_);
    nh_private.getParam("distance_error", distance_error_);

    mav_state_ = state_ptr;
}

void Initialization::execute(const Event& evt) {
    BHV_INFO("Initializing...");
    // waitForDeploy();
    // detach();
    if (!takeoff()) {
        return;
    }
}

bool Initialization::takeoff() {
    BHV_INFO("Taking off... ");

    geometry_msgs::Pose takeoff_pose = mav_state_->getPose();
    takeoff_pose.position.z = hover_height_;

    mav_state_->goToLocation(takeoff_pose);

    bool status = (fabs((mav_state_)->getPose().position.z - hover_height_) < distance_error_);
    return status;
}

}  // namespace ariitk::agent_state_machine
