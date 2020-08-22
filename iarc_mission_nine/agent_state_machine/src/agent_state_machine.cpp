#include <agent_state_machine/agent_state_machine.hpp>

namespace ariitk::agent_state_machine {

AgentStateMachine::AgentStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("poll_rate", poll_rate_);
    nh_private.getParam("verbose/state", verbose_);

    machine_.start();
    machine_.init(nh, nh_private);

    state_pub_ = nh_private.advertise<std_msgs::String>("curr_state", 1);
    state_timer_ = nh_private.createTimer(ros::Duration(poll_rate_), &AgentStateMachine::publishCurrState, this);

    // Fly!
    executeBehaviour<Initialization>();
}

void AgentStateMachine::run() {
    // First, look for the mast
    performTask<MastSearch>();  // exits when mast is found
    // Second, remove the existing block
    performTask<DetachBlock>();  // exits once block is removed
    // Now, attach payload in place of the block
    performTask<AttachBlock>();  // exits once block is in place
    // Die!
    executeBehaviour<Termination>();

    machine_.stop();
}

template<class Event>
void AgentStateMachine::performTask() {
    // since every task must come back to hover, these two calls are always together
    executeBehaviour<Event>();
    executeBehaviour<Hovering>();  // hover at the current height by default
}

void AgentStateMachine::publishCurrState(const ros::TimerEvent&) {
    ros::Rate loop_rate(poll_rate_);

    std_msgs::String state_msg;
    while (ros::ok()) {
        ros::spinOnce();
        state_msg.data = state_names[machine_.current_state()[0]];
        state_pub_.publish(state_msg);
        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine
