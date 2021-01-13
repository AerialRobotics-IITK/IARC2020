#include <agent_state_machine/behaviours/mast_search.hpp>

namespace ariitk::agent_state_machine {

void MastSearch::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr) {
    MastSearch::mast_finder_.init(nh);
    ros::Rate loop_rate(30.0);
    for (int i = 0; i < 45; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MastSearch::execute(const Event evt) {
    BHV_INFO("Searching for Mast...");
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        MastSearch::mast_finder_.run();
        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine