#include <agent_state_machine/behaviours/hovering.hpp>

namespace ariitk::agent_state_machine {

void Hovering::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr) {
    mav_state_ = state_ptr;
}

void Hovering::execute(const Event& evt) {
    // Do nothing: Controller automatically hovers
    BHV_INFO("Hovering at Location!");
    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine
