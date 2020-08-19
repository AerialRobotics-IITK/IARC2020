#pragma once

#include <carrier_state_machine/carrier_state_machine_base.hpp>
#include <future>
#include <std_msgs/String.h>

namespace ariitk::carrier_state_machine {

class CarrierStateMachine : public StateMachineBase {
  public:
    CarrierStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

  private:
    void publishCurrState();
    template<class Event>
    void performTask();

    StateMachineBackend machine_;

    ros::Publisher state_pub_;

    double poll_rate_;
    bool verbose_;
};

}  // namespace ariitk::carrier_state_machine
