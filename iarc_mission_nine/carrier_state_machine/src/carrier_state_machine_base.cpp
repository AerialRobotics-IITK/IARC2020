#include <carrier_state_machine/carrier_state_machine_base.hpp>

namespace ariitk::carrier_state_machine {

void StateMachineBase::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("verbose/transition", verbose_);
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("land_height", land_height_);
    nh_private.getParam("carrier/x", home_pose_.position.x);
    nh_private.getParam("carrier/y", home_pose_.position.y);
    nh_private.getParam("call_rate", call_rate_);
    nh_private.getParam("distance_error", dist_err_);

    home_pose_.position.z = 0;
    has_payload = true;

    odom_sub_ = nh.subscribe("odometry", 1, &StateMachineBase::odometryCallback, this);
    mav_state_sub_ = nh.subscribe("mavros/state", 1, &StateMachineBase::stateCallback, this);

    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    deploy_client_ = nh.serviceClient<std_srvs::Trigger>("agent/deploy");
}

void StateMachineBase::takeoff(const Takeoff& cmd) {
    FSM_INFO("Taking off!");

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
            break;
        }
    }

    goToPosition(mav_pose_.position.x, mav_pose_.position.y, hover_height_);
}

void StateMachineBase::reachShip(const Travel& cmd) {
    FSM_INFO("Travelling to Ship!");
    goToPosition(mav_pose_.position.x + 2.0, mav_pose_.position.y + 2.0, hover_height_);  // just for testing

    // while (ros::ok()) {
    //     // TODO: Execute planned trajectory here
    // }
}

void StateMachineBase::deployAgent(const Detach& cmd) {
    FSM_INFO("Dropping Agent...");

    has_payload = false;  // temporary shortcircuit
    std_srvs::Trigger deploy_msg;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 69;

    while (ros::ok() && has_payload) {
        loop_rate.sleep();
        deploy_client_.call(deploy_msg);
        has_payload = !deploy_msg.response.success;
        ros::spinOnce();

        if (attempts++ > max_attempts) {
            ROS_ERROR("Couldn't deploy agent!");
            break;
        }
    }
}

void StateMachineBase::returnHome(const Return& cmd) {
    FSM_INFO("Return to Land");

    // TODO: Go back home and land
}

void StateMachineBase::hover(const Hold& cmd) {
    FSM_INFO("In Position Hold");
    goToPosition(mav_pose_.position.x, mav_pose_.position.y, cmd.hold_height);
    switchMode("AUTO.LOITER");
}

void StateMachineBase::land(const Terminate& cmd) {
    FSM_INFO("Landing!");
    goToPosition(mav_pose_.position.x, mav_pose_.position.y, land_height_);
    switchMode("AUTO.LAND");
}

void StateMachineBase::switchMode(const std::string& des_mode) {
    if (mav_state_.mode == des_mode) {
        return;
    }

    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = des_mode;
    mode_msg.response.mode_sent = false;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 69;

    geometry_msgs::PoseStamped cmd_msg;

    while (ros::ok() && mav_state_.mode != des_mode) {
        loop_rate.sleep();

        cmd_msg.header.stamp = ros::Time::now();
        cmd_msg.pose = mav_pose_;
        cmd_pose_pub_.publish(cmd_msg);

        mode_client_.call(mode_msg);
        ros::spinOnce();

        if (attempts++ > max_attempts) {
            ROS_ERROR_STREAM("Couldn't switch mode to " << des_mode);
            break;
        }

        // ROS_INFO_STREAM(attempts << " " << int(mode_msg.response.mode_sent) << " " << mav_state_.mode);
    }
}

void StateMachineBase::goToPosition(const double& x, const double& y, const double& z) {
    FSM_INFO("Publishing pose setpoint ... ");
    geometry_msgs::PoseStamped cmd_msg;

    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.pose.position.x = x;
    cmd_msg.pose.position.y = y;
    cmd_msg.pose.position.z = z;

    switchMode("OFFBOARD");
    cmd_pose_pub_.publish(cmd_msg);

    ros::Rate loop_rate(call_rate_);

    FSM_INFO("Waiting to reach setpoint ... ");
    double dist = pow((mav_pose_.position.x - x), 2) + pow((mav_pose_.position.y - y), 2) + pow((mav_pose_.position.z - z), 2);
    while (ros::ok() && dist > dist_err_) {
        loop_rate.sleep();
        ros::spinOnce();
        dist = pow((mav_pose_.position.x - x), 2) + pow((mav_pose_.position.y - y), 2) + pow((mav_pose_.position.z - z), 2);
        cmd_pose_pub_.publish(cmd_msg);
        ROS_INFO_STREAM(dist);
    }
}

void StateMachineBase::odometryCallback(const nav_msgs::Odometry& odom) {
    mav_pose_ = odom.pose.pose;
    curr_height = mav_pose_.position.z;
}

void StateMachineBase::stateCallback(const mavros_msgs::State& state) {
    mav_state_ = state;
}

}  // namespace ariitk::carrier_state_machine
