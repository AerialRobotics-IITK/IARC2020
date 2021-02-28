#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#define BHV_INFO(X) ROS_INFO_STREAM("[BHV]: " << X)

namespace ariitk::carrier_state_machine {

class CarrierState {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    inline geometry_msgs::Pose getPose() {
        return pose_;
    };

    inline mavros_msgs::State getState() {
        return state_;
    };

    bool switchMode(const std::string& des_mode);

  private:
    void odometryCallback(const nav_msgs::Odometry& odom);
    void stateCallback(const mavros_msgs::State& state);

    ros::Subscriber odom_sub_;
    ros::Subscriber state_sub_;

    ros::ServiceClient mode_client_;

    geometry_msgs::Pose pose_;
    mavros_msgs::State state_;

    double call_rate_;
};

}  // namespace ariitk::carrier_state_machine
