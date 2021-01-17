#pragma once

#include <carrier_state_machine/carrier_state/carrier_state.hpp>

namespace ariitk::carrier_state_machine {

class HoveringAfterDetach {
  public:
    struct Event {};

    void execute(const Event& evt);
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr);

  private:
    std::shared_ptr<CarrierState> mav_state_;
};

}  // namespace ariitk::carrier_state_machine
