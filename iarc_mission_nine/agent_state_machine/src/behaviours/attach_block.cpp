#include <agent_state_machine/behaviours/attach_block.hpp>

namespace ariitk::agent_state_machine {

void AttachBlock::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr) {
    attach_client_ = nh.ServiceClient<gazebo_ros_link_attacher::AttacherRequest>("link_attacher_node/attach");

    bool flag = false;

    req_.model_name_1 = "gripper";
    req_.link_name_1 "left_short_joint";
    req_.model_name_2 = "communications_module";
    req_.link_name_2 = "link";
}

void AttachBlock::execute(const Event evt) {
    BHV_INFO("Placing payload...");
    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        if (flag == false) {
            attach_client_.call(req_);
            flag = true;
        }
        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine