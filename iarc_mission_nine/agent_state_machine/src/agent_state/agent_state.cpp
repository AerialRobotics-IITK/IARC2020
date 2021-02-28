#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {
AgentState::AgentState(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.param("call_rate", call_rate_, 30.0);
    nh_private.param("distance_error", distance_error_, 0.1);

    odom_sub_ = nh.subscribe("odometry", 1, &AgentState::odometryCallback, this);
    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

    // wait for first message
    nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>("odometry", ros::Duration(1));
    if (odom == NULL) {
        ROS_ERROR("No odometry received!");
    } else {
        pose_ = odom->pose.pose;
    }
}

void AgentState::odometryCallback(const nav_msgs::Odometry& odom) {
    pose_ = odom.pose.pose;
}

void AgentState::goToLocation(const geometry_msgs::Pose& des_pose) {
    publishPoseSetpoint(des_pose);

    ros::Rate loop_rate(call_rate_);
    while (ros::ok() && (pointDistance(pose_.position, des_pose.position) > distance_error_)) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void AgentState::publishPoseSetpoint(const geometry_msgs::Pose& des_pose) {
    geometry_msgs::PoseStamped cmd_msg;
    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.pose = des_pose;
    cmd_pose_pub_.publish(cmd_msg);
}

}  // namespace ariitk::agent_state_machine
