#pragma once

#include <state_machine_definition/state_transition_behaviour.hpp>

namespace ariitk::agent_state_machine {

class DetachBlock : public ariitk::state_machine::Behaviour {
  public:
    typedef ariitk::state_machine::Behaviour::Event Event;

    void execute(const Event& evt) override;
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
};

}  // namespace ariitk::agent_state_machine
