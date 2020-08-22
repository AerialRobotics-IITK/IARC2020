#pragma once

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

#include <state_machine_def/state_transition_behaviour.hpp>

namespace ariitk::agent_state_machine {

class Initialization : public ariitk::state_machine::Behaviour {
  public:
    typedef ariitk::state_machine::Behaviour::Event Event;

    void action(const Event& evt) override;
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    void odometryCallback(const nav_msgs::Odometry& odom);
    void stateCallback(const mavros_msgs::State& state);
    bool arm();
    bool takeoff();

    mavros_msgs::State mav_state_;
    geometry_msgs::Pose mav_pose_;

    ros::Subscriber odom_sub_;
    ros::Subscriber mav_state_sub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient takeoff_client_;

    double call_rate_;
    double hover_height_;
};

}  // namespace ariitk::agent_state_machine
