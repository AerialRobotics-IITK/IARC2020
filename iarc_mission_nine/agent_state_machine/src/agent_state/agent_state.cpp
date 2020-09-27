#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {
void AgentState::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("call_rate", call_rate_);
    nh_private.getParam("distance_error", distance_error_);

    odom_sub_ = nh.subscribe("odometry", 1, &AgentState::odometryCallback, this);
    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

    pose_.position.z = DBL_MIN;
    ros::Rate loop_rate(20);

    // wait for first message
    while (pose_.position.z == DBL_MIN) {
        ros::spinOnce();
        loop_rate.sleep();
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