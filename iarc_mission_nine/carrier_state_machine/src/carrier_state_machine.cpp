#include <carrier_state_machine/carrier_state_machine.hpp>

namespace ariitk::carrier_state_machine {

CarrierStateMachine::CarrierStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("poll_rate", poll_rate_);
    nh_private.getParam("verbose/state", verbose_);

    machine_.start();
    machine_.init(nh, nh_private);

    state_pub_ = nh_private.advertise<std_msgs::String>("curr_state", 1);
    state_timer_ = nh_private.createTimer(ros::Duration(poll_rate_), &CarrierStateMachine::publishCurrState, this);

    // Fly!
    executeBehaviour<Takeoff>();
}

void CarrierStateMachine::run() {
    // First, get to the ship
    performTask<ReachShip>();  // exits when planned trajectory is complete
    // performTask<HoveringBeforeDetach>();
    // Second, deploy the agent to do its work
    // performTask<DeployAgent>();  // exits once agent has detached
    // performTask<HoveringAfterDetach>();
    // Now, return to takeoff zone
    // performTask<ReturnHome>();  // exits once carrier is in takeoff zone
    // performTask<HoveringBeforeDetach>();
    // Die!
    // executeBehaviour<Termination>();

    machine_.stop();
}

template<class Event> void CarrierStateMachine::performTask() {
    // since every task must come back to hover, these two calls are always together
    executeBehaviour<Event>();
    // executeBehaviour<Hovering>();  // hover at the current height by default
}

void CarrierStateMachine::publishCurrState(const ros::TimerEvent&) {
    ros::Rate loop_rate(poll_rate_);

    std_msgs::String state_msg;
    while (ros::ok()) {
        ros::spinOnce();
        state_msg.data = state_names[machine_.current_state()[0]];
        state_pub_.publish(state_msg);
        loop_rate.sleep();
    }
}

}  // namespace ariitk::carrier_state_machine
