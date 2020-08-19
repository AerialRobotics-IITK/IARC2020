#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <state_machine_def/state_machine_def.hpp>

namespace ariitk::agent_state_machine {

class StateMachineBase : public ariitk::state_machine::FSMDef<StateMachineBase> {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

    // State names
    std::vector<std::string> state_names = {"Rest", "Hover", "Explore", "Detach", "Attach"};

    // State definitions
    struct Rest : public State<Rest> {};
    struct Hover : public State<Hover> {};
    struct Detach : public State<Detach> {};
    struct Attach : public State<Attach> {};
    struct Explore : public State<Explore> {};

    typedef Rest initial_state;

    // Transition events
    struct Initialize : public Command {};
    struct Search : public Command {};
    struct RemoveBlock : public Command {};
    struct PlaceBlock : public Command {};
    struct Terminate : public Command {};
    struct Hold : public Command {
        Hold(const double& height)
            : hold_height(height) {
        }
        double hold_height;
    };

    // Guard variables
    bool has_payload;
    bool mast_detected;
    double curr_height;

    // Transition Guards
    template<class Event>
    bool isMastVisible(const Event& cmd) {
        return mast_detected;
    }
    bool needMastSearch(const Search& cmd) {
        return !mast_detected;
    }
    bool hasNoPayload(const Hold& cmd) {
        return !has_payload;
    }
    bool canAttachBlock(const PlaceBlock& cmd) {
        return (has_payload && mast_detected);
    }

    // Transition actions
    void initialize(const Initialize& cmd);
    void findMast(const Search& cmd);
    void hover(const Hold& cmd);
    void detachBlock(const RemoveBlock& cmd);
    void attachBlock(const PlaceBlock& cmd);
    void land(const Terminate& cmd);

    // clang-format off
    struct transition_table
        : boost::mpl::vector<
              //      Type     Start          Event          Next            Action                         Guard
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                     a_row<    Rest     ,  Initialize   ,  Hover    ,  &StateMachineBase::initialize                                             >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Hover    ,  Search       ,  Explore  ,  &StateMachineBase::findMast     ,  &StateMachineBase::needMastSearch   >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Explore  ,  Hold         ,  Hover    ,  &StateMachineBase::hover        ,  &StateMachineBase::isMastVisible    >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Hover    ,  RemoveBlock  ,  Detach   ,  &StateMachineBase::detachBlock  ,  &StateMachineBase::isMastVisible    >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                     a_row<    Detach   ,  Hold         ,  Hover    ,  &StateMachineBase::hover                                               >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Hover    ,  PlaceBlock   ,  Attach   ,  &StateMachineBase::attachBlock  ,  &StateMachineBase::canAttachBlock   >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Attach   ,  Hold         ,  Hover    ,  &StateMachineBase::hover        ,  &StateMachineBase::hasNoPayload     >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                     a_row<    Hover    ,  Terminate    ,  Rest     ,  &StateMachineBase::land                                                >
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
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

}  // namespace ariitk::agent_state_machine
