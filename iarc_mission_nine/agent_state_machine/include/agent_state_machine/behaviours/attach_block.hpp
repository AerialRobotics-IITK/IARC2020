#pragma once

#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {

class AttachBlock {
  public:
    struct Event {};

    void execute(const Event& evt);
<<<<<<< HEAD
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr);
=======
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr);
>>>>>>> a3d2e67753f208637e1166982431f4a5d133d9e4

  private:
    std::shared_ptr<AgentState> mav_state_;
};

<<<<<<< HEAD
}  // namespace ariitk::agent_state_machine
=======
}  // namespace ariitk::agent_state_machine
>>>>>>> a3d2e67753f208637e1166982431f4a5d133d9e4
