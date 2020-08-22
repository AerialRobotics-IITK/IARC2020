#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace ariitk::agent_state_machine {

class AgentState {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    inline geometry_msgs::Pose getPose() {
        return pose_;
    };

    inline mavros_msgs::State getState() {
        return state_;
    };

  private:
    void odometryCallback(const nav_msgs::Odometry& odom);
    void stateCallback(const mavros_msgs::State& state);

    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;

    geometry_msgs::Pose pose_;
    mavros_msgs::State state_;
};

}  // namespace ariitk::agent_state_machine
