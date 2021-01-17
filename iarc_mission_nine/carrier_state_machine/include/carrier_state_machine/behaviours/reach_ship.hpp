#pragma once

#include <carrier_state_machine/carrier_state/carrier_state.hpp>
#include <path_manager_base.h>  //for rosplane

namespace ariitk::carrier_state_machine {

class ReachShip {
  public:
    struct Event {};

    void execute(const Event& evt);
    void init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr);

  private:
    std::shared_ptr<CarrierState> mav_state_;
    rosplane::path_manager_base* est = new rosplane::path_manager_example();
};

}  // namespace ariitk::carrier_state_machine
