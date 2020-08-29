#include <agent_state_machine/behaviours/initialization.hpp>

namespace ariitk::agent_state_machine {

void Initialization::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr) {
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("call_rate", call_rate_);
    nh_private.getParam("distance_error", distance_error_);

    mav_state_ = state_ptr;

    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    // takeoff_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
}

void Initialization::execute(const Event& evt) {
    BHV_INFO("Initializing...");
    // waitForDeploy();
    if (!arm()) {
        return;
    }
    // detach();
    if (!takeoff()) {
        return;
    }

    // wait for takeoff to complete
    ros::Rate loop_rate(call_rate_);
    while (ros::ok() && (mav_state_->getPose().position.z < hover_height_ - distance_error_)) {
        ROS_INFO_STREAM("WAIT");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// TODO: send arming toggle into mav state
bool Initialization::arm() {
    BHV_INFO("Arming... ");

    mavros_msgs::CommandBool arm_msg;
    arm_msg.request.value = true;
    arm_msg.response.success = false;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 69;

    while (ros::ok() && arm_msg.response.success != true && mav_state_->getState().armed != true) {
        arming_client_.call(arm_msg);
        ros::spinOnce();

        if (attempts++ > max_attempts) {
            ROS_FATAL("Arming unsuccessful!");
            return false;
        }

        loop_rate.sleep();
    }
    return true;
}

bool Initialization::takeoff() {
    BHV_INFO("Switching to OFFBOARD for takeoff... ");

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
