#pragma once

#include <future>
#include <std_msgs/String.h>

#include <agent_state_machine/agent_state_machine_base.hpp>

namespace ariitk::agent_state_machine {

class AgentStateMachine : public StateMachineBase {
  public:
    AgentStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

  private:
    void publishCurrState();

    template<class Event>
    void performTask();

    template<class Behaviour>
    inline void executeBehaviour() {
        machine_.process_event(Behaviour::Event());
    }

    StateMachineBackend machine_;

    ros::Publisher state_pub_;
    ros::Timer state_timer_;

    double poll_rate_;
    bool verbose_;
};

}  // namespace ariitk::agent_state_machine
