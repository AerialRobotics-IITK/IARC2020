#include <agent_state_machine/agent_state_machine_base.hpp>

namespace ariitk::agent_state_machine {

void StateMachineBase::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("verbose/transition", verbose_);  // TODO: Use these flags

    has_payload = true;
    mast_detected = false;

    state_ptr_ = std::make_shared<AgentState>(AgentState());
    state_ptr_->init(nh, nh_private);

    // initialize behaviours
    initializeBehaviours(nh, nh_private);
}

void StateMachineBase::initializeBehaviours(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    init_behaviour_.init(nh, nh_private, state_ptr_);
    // hover_behaviour_.init(nh, nh_private, state_ptr_);
    // search_behaviour_.init(nh, nh_private, state_ptr_);
    // detach_behaviour_.init(nh, nh_private, state_ptr_);
    // attach_behaviour_.init(nh, nh_private, state_ptr_);
    // land_behaviour_.init(nh, nh_private, state_ptr_);
}

void StateMachineBase::initialize(const Initialization::Event& cmd) {
    FSM_INFO("Taking off!");
    init_behaviour_.execute(cmd);
}

void StateMachineBase::findMast(const MastSearch::Event& cmd) {
    FSM_INFO("Searching for Mast...");
    search_behaviour_.execute(cmd);
    mast_detected = true;  // temporary shortcircuit
}

void StateMachineBase::detachBlock(const DetachBlock::Event& cmd) {
    FSM_INFO("Removing block on mast...");
    detach_behaviour_.execute(cmd);
}

void StateMachineBase::attachBlock(const AttachBlock::Event& cmd) {
    FSM_INFO("Placing block on mast...");
    attach_behaviour_.execute(cmd);
    has_payload = false;  // temporary shortcircuit
}

void StateMachineBase::hover(const Hovering::Event& cmd) {
    FSM_INFO("In Position Hold");
    hover_behaviour_.execute(cmd);
}

void StateMachineBase::land(const Termination::Event& cmd) {
    FSM_INFO("Landing!");
    land_behaviour_.execute(cmd);
}

}  // namespace ariitk::agent_state_machine
