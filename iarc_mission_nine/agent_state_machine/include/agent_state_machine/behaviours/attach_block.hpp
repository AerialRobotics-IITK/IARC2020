#pragma once

#include <agent_state_machine/agent_state/agent_state.hpp>
#include <state_machine_definition/state_transition_behaviour.hpp>

namespace ariitk::agent_state_machine {

class AttachBlock : public ariitk::state_machine::Behaviour {
  public:
    typedef ariitk::state_machine::Behaviour::Event Event;

    void execute(const Event& evt) override;
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    std::shared_ptr<AgentState> mav_state_;
};

}  // namespace ariitk::agent_state_machine
