#pragma once

#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {

class MastSearch {
  public:
    struct Event {};

    void execute(const Event& evt);
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    std::shared_ptr<AgentState> mav_state_;  // TODO: Create Destructors
};

}  // namespace ariitk::agent_state_machine