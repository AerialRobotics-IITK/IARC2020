#include <carrier_state_machine/behaviours/reach_ship.hpp>

namespace ariitk::carrier_state_machine {

void ReachShip::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
    rosplane_.init(nh);
}

void ReachShip::execute(const Event& evt) {
    rosplane_.run();
}

}  // namespace ariitk::carrier_state_machine
