#include <carrier_state_machine/carrier_state_machine_base.hpp>

namespace ariitk::carrier_state_machine {

void StateMachineBase::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("verbose/transition", verbose_);
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("land_height", land_height_);

    has_payload = true;

    odom_sub_ = nh.subscribe("odometry", 1, &StateMachineBase::odometryCallback, this);
    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
}

void StateMachineBase::takeoff(const Takeoff& cmd) {
    echo("Taking off!");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, hover_height_);
}

void StateMachineBase::reachShip(const Travel& cmd) {
    echo("Travelling to Ship!");
    while (ros::ok()) {
        // TODO: Execute planned trajectory here
    }
}

void StateMachineBase::deployAgent(const Detach& cmd) {
    echo("Dropping Agent...");
    has_payload = false;
    while (ros::ok() && has_payload) {
        // TODO: Deploy the agent quadrotor
    }
}

void StateMachineBase::returnHome(const Return& cmd) {
    echo("Return to Land");
    // TODO: Go back home and land
}

void StateMachineBase::hover(const Hold& cmd) {
    echo("In Position Hold");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, cmd.hold_height);
}

void StateMachineBase::land(const Terminate& cmd) {
    echo("Landing!");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, land_height_);
}

void StateMachineBase::publishPoseCommand(const double& x, const double& y, const double& z) {
    geometry_msgs::PoseStamped cmd_msg;

    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.pose.position.x = x;
    cmd_msg.pose.position.y = y;
    cmd_msg.pose.position.z = z;

    cmd_pose_pub_.publish(cmd_msg);
}

void StateMachineBase::odometryCallback(const nav_msgs::Odometry& odom) {
    mav_pose_ = odom.pose.pose;
    curr_height = mav_pose_.position.z;
}

}  // namespace ariitk::carrier_state_machine
