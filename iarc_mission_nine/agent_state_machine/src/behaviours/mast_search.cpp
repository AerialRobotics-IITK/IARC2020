#include <agent_state_machine/behaviours/mast_search.hpp>

namespace ariitk::agent_state_machine {

void MastSearch::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr) {
    MastSearch::plate_detector_.init(nh);
    MastSearch::pose_estimator_.init(nh, nh_private);
    MastSearch::mast_finder_.init(nh);
}

void MastSearch::execute(const Event evt) {
    BHV_INFO("Searching for Mast...");
    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        for (int i = 0; i < 2; i++) {
            MastSearch::plate_detector_.run();
            ros::spinOnce();
            // loop_rate.sleep();
            MastSearch::pose_estimator_.run();
            ros::spinOnce();
        }

        MastSearch::mast_finder_.run();
        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine