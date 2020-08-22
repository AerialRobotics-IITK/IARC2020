#include <carrier_state_machine/carrier_state_machine.hpp>

namespace ariitk::carrier_state_machine {

CarrierStateMachine::CarrierStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("poll_rate", poll_rate_);
    nh_private.getParam("verbose/state", verbose_);

    machine_.start();
    machine_.init(nh, nh_private);

    state_pub_ = nh_private.advertise<std_msgs::String>("curr_state", 1);

    // Fly!
    machine_.process_event(Takeoff());
}

void CarrierStateMachine::spin() {
    auto state_publish_thread = std::async(std::launch::async, [this] { publishCurrState(); });

    // First, get to the ship
    performTask<Travel>();  // exits when planned trajectory is complete
    // Second, deploy the agent to do its work
    performTask<Detach>();  // exits once agent has detached
    // Now, return to takeoff zone
    performTask<Return>();  // exits once carrier is in takeoff zone
    // Die!
    machine_.process_event(Terminate());

    machine_.stop();
}

template<class Event>
void CarrierStateMachine::performTask() {
    // since every task must come back to hover, these two calls are always together
    machine_.process_event(Event());
    machine_.process_event(Hold(curr_height));  // hover at the current height by default
}

void CarrierStateMachine::publishCurrState() {
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
