#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <state_machine_def/state_machine_def.hpp>

namespace ariitk::carrier_state_machine {

class StateMachineBase : public ariitk::state_machine::FSMDef<StateMachineBase> {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

    // State names
    std::vector<std::string> state_names = {"Rest", "Hover", "Mission", "Deploy"};

    // State definitions
    struct Rest : public State<Rest> {};
    struct Hover : public State<Hover> {};
    struct Deploy : public State<Deploy> {};
    struct Mission : public State<Mission> {};

    typedef Rest initial_state;

    // Transition events
    struct Takeoff : public Command {};
    struct Travel : public Command {};
    struct Detach : public Command {};
    struct Return : public Command {};
    struct Terminate : public Command {};
    struct Hold : public Command {
        Hold(const double& height)
            : hold_height(height) {
        }
        double hold_height;
    };

    // Guard variables
    bool has_payload;
    double curr_height;

    // Transition Guards
    bool isAgentDeployed(const Return& cmd) {
        return !has_payload;
    }
    bool hasAgent(const Detach& cmd) {
        return has_payload;
    }

    // Transition actions
    void takeoff(const Takeoff& cmd);
    void reachShip(const Travel& cmd);
    void hover(const Hold& cmd);
    void deployAgent(const Detach& cmd);
    void returnHome(const Return& cmd);
    void land(const Terminate& cmd);

    // clang-format off
    struct transition_table
        : boost::mpl::vector<
              //      Type     Start          Event          Next            Action                         Guard
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Rest     ,  Takeoff      ,  Hover    ,  &StateMachineBase::takeoff                                              >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Hover    ,  Travel       ,  Mission  ,  &StateMachineBase::reachShip                                            >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Mission  ,  Hold         ,  Hover    ,  &StateMachineBase::hover                                                >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                       row<    Hover    ,  Detach       ,  Deploy   ,  &StateMachineBase::deployAgent  ,  &StateMachineBase::hasAgent          >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Deploy   ,  Hold         ,  Hover    ,  &StateMachineBase::hover                                                >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                       row<    Hover    ,  Return       ,  Mission  ,  &StateMachineBase::returnHome   ,  &StateMachineBase::isAgentDeployed   >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Hover    ,  Terminate    ,  Rest     ,  &StateMachineBase::land                                                 >
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
              > {};
    // clang-format on

  private:
    void publishPoseCommand(const double& x, const double& y, const double& z);
    void odometryCallback(const nav_msgs::Odometry& odom);

    geometry_msgs::Pose mav_pose_;

    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pose_pub_;

    bool verbose_;

    double hover_height_;
    double land_height_;
};

}  // namespace ariitk::carrier_state_machine
