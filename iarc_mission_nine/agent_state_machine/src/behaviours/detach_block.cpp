#include <agent_state_machine/behaviours/detach_block.hpp>

namespace ariitk::agent_state_machine {

void DetachBlock::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr) {
}

void DetachBlock::execute(const Event& evt) {
    BHV_INFO("Detaching module...");
    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine
