#include <agent_state_machine/behaviours/attach_block.hpp>

namespace ariitk::agent_state_machine {

void AttachBlock::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr) {
}

void AttachBlock::execute(const Event& evt) {
    BHV_INFO("Placing payload...");
    // ros::Rate loop_rate(30.0);
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

}  // namespace ariitk::agent_state_machine
