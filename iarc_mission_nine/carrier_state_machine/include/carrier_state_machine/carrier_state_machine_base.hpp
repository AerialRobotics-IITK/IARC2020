#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include <carrier_state_machine/behaviours/deploy_agent.hpp>
#include <carrier_state_machine/behaviours/hovering.hpp>
#include <carrier_state_machine/behaviours/reach_ship.hpp>
#include <carrier_state_machine/behaviours/return_home.hpp>
#include <carrier_state_machine/behaviours/takeoff.hpp>
#include <carrier_state_machine/behaviours/termination.hpp>

#include <carrier_state_machine/carrier_state/carrier_state.hpp>
#include <state_machine_definition/state_machine.hpp>

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

    // Guard variables
    bool has_payload;
    double curr_height;

    // Transition Guards
    bool isAgentDeployed(const ReturnHome::Event& cmd) {
        return !has_payload;
    }
    bool hasAgent(const DeployAgent::Event& cmd) {
        return has_payload;
    }

    // Transition actions
    void takeoff(const Takeoff::Event& cmd);
    void reachShip(const ReachShip::Event& cmd);
    void hover(const Hovering::Event& cmd);
    void deployAgent(const DeployAgent::Event& cmd);
    void returnHome(const ReturnHome::Event& cmd);
    void land(const Termination::Event& cmd);

    // clang-format off
    struct transition_table
        : boost::mpl::vector<
              //      Type     Start          Event          Next            Action                         Guard
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Rest     ,  Takeoff::Event      ,  Hover    ,  &StateMachineBase::takeoff                                              >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Hover    ,  ReachShip::Event       ,  Mission  ,  &StateMachineBase::reachShip                                            >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Mission  ,  Hovering::Event         ,  Hover    ,  &StateMachineBase::hover                                                >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                       row<    Hover    ,  DeployAgent::Event       ,  Deploy   ,  &StateMachineBase::deployAgent  ,  &StateMachineBase::hasAgent          >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Deploy   ,  Hovering::Event         ,  Hover    ,  &StateMachineBase::hover                                                >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                       row<    Hover    ,  ReturnHome::Event       ,  Mission  ,  &StateMachineBase::returnHome   ,  &StateMachineBase::isAgentDeployed   >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
                     a_row<    Hover    ,  Termination::Event    ,  Rest     ,  &StateMachineBase::land                                                 >
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ------------------------------------ +++
              > {};
    // clang-format on

  private:
    void initializeBehaviours(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    bool verbose_;
    std::shared_ptr<CarrierState> state_ptr_;

    // behaviour objects
    Takeoff takeoff_behaviour_;
    ReachShip travel_behaviour_;
    ReturnHome return_behaviour_;
    Termination land_behaviour_;
    Hovering hover_behaviour_;
    DeployAgent deploy_behaviour_;
};

}  // namespace ariitk::carrier_state_machine
