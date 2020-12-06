#pragma once

#include <future>
#include <std_msgs/String.h>

#include <carrier_state_machine/carrier_state_machine_base.hpp>

namespace ariitk::carrier_state_machine {

class CarrierStateMachine : public StateMachineBase {
  public:
    CarrierStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();

  private:
    void publishCurrState(const ros::TimerEvent&);

    template<class Event>
    void performTask();

    template<class Behaviour>
    inline void executeBehaviour() {
        machine_.process_event(typename Behaviour::Event());
    }

    StateMachineBackend machine_;

    ros::Publisher state_pub_;
    ros::Timer state_timer_;

    double poll_rate_;
    bool verbose_;
};

}  // namespace ariitk::carrier_state_machine
