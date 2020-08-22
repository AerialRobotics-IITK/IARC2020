#include <agent_state_machine/behaviours/termination.hpp>

namespace ariitk::agent_state_machine {

void Termination::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr) {
}

void Termination::execute(const Event& evt) {
}

}  // namespace ariitk::agent_state_machine
