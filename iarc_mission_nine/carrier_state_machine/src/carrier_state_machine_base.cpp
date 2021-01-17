#include <carrier_state_machine/carrier_state_machine_base.hpp>

namespace ariitk::carrier_state_machine {

void StateMachineBase::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("verbose/transition", verbose_);  // TODO: Use these flags

    has_payload = true;

    state_ptr_ = std::make_shared<CarrierState>(CarrierState());
    state_ptr_->init(nh, nh_private);

    // initialize behaviours
    initializeBehaviours(nh, nh_private);
}

void StateMachineBase::initializeBehaviours(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    takeoff_behaviour_.init(nh, nh_private, state_ptr_);
    hover_before_detach_behaviour_.init(nh, nh_private, state_ptr_);
    hover_after_detach_behaviour_.init(nh, nh_private, state_ptr_);
    travel_behaviour_.init(nh, nh_private, state_ptr_);
    return_behaviour_.init(nh, nh_private, state_ptr_);
    deploy_behaviour_.init(nh, nh_private, state_ptr_);
    land_behaviour_.init(nh, nh_private, state_ptr_);
}

void StateMachineBase::takeoff(const Takeoff::Event& cmd) {
    FSM_INFO("Taking off!");
    takeoff_behaviour_.execute(cmd);
}

void StateMachineBase::reachShip(const ReachShip::Event& cmd) {
    FSM_INFO("Travelling to Ship!");
    travel_behaviour_.execute(cmd);
}

void StateMachineBase::deployAgent(const DeployAgent::Event& cmd) {
    FSM_INFO("Dropping Agent...");
    deploy_behaviour_.execute(cmd);
    has_payload = false;  // temporary shortcircuit
}

void StateMachineBase::returnHome(const ReturnHome::Event& cmd) {
    FSM_INFO("Return to Land");
    return_behaviour_.execute(cmd);
}

void StateMachineBase::hover(const HoveringBeforeDetach::Event& cmd) {
    FSM_INFO("In Position Hold");
    hover_before_detach_behaviour_.execute(cmd);
}

void StateMachineBase::hover(const HoveringAfterDetach::Event& cmd) {
    FSM_INFO("In Position Hold");
    hover_after_detach_behaviour_.execute(cmd);
}


void StateMachineBase::land(const Termination::Event& cmd) {
    FSM_INFO("Landing!");
    land_behaviour_.execute(cmd);
}

}  // namespace ariitk::carrier_state_machine
