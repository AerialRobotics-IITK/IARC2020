#include <agent_transition_behaviours/initialization.hpp>

namespace ariitk::agent_state_machine {

void Initialization::init(ros::NodeHandle nh, ros::NodeHandle nh_private) {
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("call_rate", call_rate_);

    odom_sub_ = nh.subscribe("odometry", 1, &Initialization::odometryCallback, this);
    mav_state_sub_ = nh.subscribe("mavros/state", 1, &Initialization::stateCallback, this);

    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/set_mode");
}

void Initialization::action(const Event& evt) {
    init(evt.nh, evt.nh_private);
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
    while (ros::ok() && (mav_pose_.position.z < hover_height_ - 0.05)) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool Initialization::arm() {
    BHV_INFO("Arming... ");

    mavros_msgs::CommandBool arm_msg;
    arm_msg.request.value = true;
    arm_msg.response.success = false;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 69;

    while (ros::ok() && arm_msg.response.success != true && mav_state_.armed != true) {
        loop_rate.sleep();
        arming_client_.call(arm_msg);
        ros::spinOnce();

        if (attempts++ > max_attempts) {
            ROS_FATAL("Arming unsuccessful!");
            return false;
        }
    }

    return true;
}

bool Initialization::takeoff() {
    BHV_INFO("Switching to AUTO.TAKEOFF... ");

    mavros_msgs::CommandTOL takeoff_msg;
    takeoff_msg.request.altitude = hover_height_;
    takeoff_msg.response.success = false;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 69;

    while (ros::ok() && takeoff_msg.response.success != true && mav_state_.mode != "AUTO.TAKEOFF") {
        loop_rate.sleep();
        takeoff_client_.call(takeoff_msg);
        ros::spinOnce();

        if (attempts++ > max_attempts) {
            ROS_FATAL("Takeoff unsuccessful!");
            return false;
        }
    }

    return true;
}

void Initialization::odometryCallback(const nav_msgs::Odometry& odom) {
    mav_pose_ = odom.pose.pose;
    BHV_INFO(mav_pose_.position.z);
}

void Initialization::stateCallback(const mavros_msgs::State& state) {
    mav_state_ = state;
}

}  // namespace ariitk::agent_state_machine
