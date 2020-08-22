#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {

void AgentState::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    odom_sub_ = nh.subscribe("odometry", 1, &AgentState::odometryCallback, this);
    state_sub_ = nh.subscribe("mavros/state", 1, &AgentState::stateCallback, this);
}

void AgentState::odometryCallback(const nav_msgs::Odometry& odom) {
    pose_ = odom.pose.pose;
}

void AgentState::stateCallback(const mavros_msgs::State& state) {
    state_ = state;
}

}  // namespace ariitk::agent_state_machine
