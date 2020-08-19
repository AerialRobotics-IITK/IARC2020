#include <agent_state_machine/agent_state_machine.hpp>

using namespace ariitk::agent_state_machine;

int main(int argc, char** argv) {
    ros::init(argc, argv, "agent_state_machine");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    AgentStateMachine fsm(nh, nh_private);
    fsm.spin();
}
