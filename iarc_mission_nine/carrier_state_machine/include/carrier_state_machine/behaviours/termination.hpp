#pragma once

#include <carrier_state_machine/carrier_state/carrier_state.hpp>
#include <state_machine_definition/state_transition_behaviour.hpp>

namespace ariitk::carrier_state_machine {

class Termination {
  public:
    typedef ariitk::state_machine::Behaviour::Event Event;

    void execute(const Event& evt);
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr);

  private:
    std::shared_ptr<CarrierState> mav_state_;
};

}  // namespace ariitk::carrier_state_machine
