#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <agent_state_machine/behaviours/attach_block.hpp>
#include <agent_state_machine/behaviours/detach_block.hpp>
#include <agent_state_machine/behaviours/hovering.hpp>
#include <agent_state_machine/behaviours/initialization.hpp>
#include <agent_state_machine/behaviours/mast_search.hpp>
#include <agent_state_machine/behaviours/termination.hpp>

#include <state_machine_definition/state_machine.hpp>

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

    // Guard variables
    bool has_payload;
    bool mast_detected;

    // Transition Guards
    template<class Event>
    bool isMastVisible(const Event& cmd) {
        return mast_detected;
    }
    bool needMastSearch(const MastSearch::Event& cmd) {
        return !mast_detected;
    }
    bool hasNoPayload(const Hovering::Event& cmd) {
        return !has_payload;
    }
    bool canAttachBlock(const AttachBlock::Event& cmd) {
        return (has_payload && mast_detected);
    }

    // Transition actions --- behaviour wrappers
    // MSM Transition table expects actions to be of the same class
    // So we wrap the behaviour executors in these member functions

    void initialize(const Initialization::Event& cmd);
    void findMast(const MastSearch::Event& cmd);
    void hover(const Hovering::Event& cmd);
    void detachBlock(const DetachBlock::Event& cmd);
    void attachBlock(const AttachBlock::Event& cmd);
    void land(const Termination::Event& cmd);

    // clang-format off
    struct transition_table
        : boost::mpl::vector<
              //      Type     Start          Event          Next            Action                         Guard
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                     a_row<    Rest     ,  Initialization::Event   ,  Hover    ,  &StateMachineBase::initialize                                            >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Hover    ,  MastSearch::Event       ,  Explore  ,  &StateMachineBase::findMast     ,  &StateMachineBase::needMastSearch   >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Explore  ,  Hovering::Event         ,  Hover    ,  &StateMachineBase::hover        ,  &StateMachineBase::isMastVisible    >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Hover    ,  DetachBlock::Event  ,  Detach   ,  &StateMachineBase::detachBlock  ,  &StateMachineBase::isMastVisible    >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                     a_row<    Detach   ,  Hovering::Event        ,  Hover    ,  &StateMachineBase::hover                                               >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Hover    ,  AttachBlock::Event   ,  Attach   ,  &StateMachineBase::attachBlock  ,  &StateMachineBase::canAttachBlock   >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                       row<    Attach   ,  Hovering::Event         ,  Hover    ,  &StateMachineBase::hover        ,  &StateMachineBase::hasNoPayload     >,
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
                     a_row<    Hover    ,  Termination::Event    ,  Rest     ,  &StateMachineBase::land                                                >
              // +++ ------ + --------- + ------------- + --------- + -------------------------------- + ----------------------------------- +++
              > {};
    // clang-format on

  private:
    void initializeBehaviours(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    bool verbose_;

    // behaviour objects
    Initialization init_behaviour_;
    Hovering hover_behaviour_;
    MastSearch search_behaviour_;
    DetachBlock detach_behaviour_;
    AttachBlock attach_behaviour_;
    Termination land_behaviour_;
};

}  // namespace ariitk::agent_state_machine
