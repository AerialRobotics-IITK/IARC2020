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

    // // wait for takeoff to complete
    // ros::Rate loop_rate(call_rate_);
    // while (ros::ok() && (mav_state_->getPose().position.z < hover_height_ - distance_error_)) {
    //     ROS_INFO_STREAM("WAIT");
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

bool Initialization::takeoff() {
    BHV_INFO("Taking off... ");

    // ros::Rate loop_rate(call_rate_);
    // uint attempts = 0, max_attempts = 69;

    // // TODO: Try offboard instead of takeoff service
    // while (ros::ok() && takeoff_msg.response.success != true && mav_state_->getState().mode != "AUTO.TAKEOFF") {
    //     takeoff_client_.call(takeoff_msg);
    //     ros::spinOnce();

    //     if (attempts++ > max_attempts) {
    //         ROS_FATAL("Takeoff unsuccessful!");
    //         return false;
    //     }

    //     loop_rate.sleep();
    // }
    geometry_msgs::Pose takeoff_pose = mav_state_->getPose();
    takeoff_pose.position.z = hover_height_;

    mav_state_->goToLocation(takeoff_pose);
    // mav_state_->switchMode("AUTO.LOITER");

    bool status = (fabs((mav_state_)->getPose().position.z - hover_height_) < distance_error_);
    return status;
}

}  // namespace ariitk::agent_state_machine
