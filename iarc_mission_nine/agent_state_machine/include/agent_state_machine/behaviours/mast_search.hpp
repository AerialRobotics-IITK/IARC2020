#pragma once

#include <agent_state_machine/agent_state/agent_state.hpp>

<<<<<<< HEAD
=======
#include <mast_finder/mast_finder.hpp>

>>>>>>> a3d2e67753f208637e1166982431f4a5d133d9e4
namespace ariitk::agent_state_machine {

class MastSearch {
  public:
    struct Event {};

    void execute(const Event& evt);
<<<<<<< HEAD
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    std::shared_ptr<AgentState> mav_state_;  // TODO: Create Destructors
};

}  // namespace ariitk::agent_state_machine
=======
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    std::shared_ptr<AgentState> mav_state_;  // TODO: Create Destructors

    iarc2020::mast_locator::MastLocatorNode mast_finder_;
};

}  // namespace ariitk::agent_state_machine
>>>>>>> a3d2e67753f208637e1166982431f4a5d133d9e4
