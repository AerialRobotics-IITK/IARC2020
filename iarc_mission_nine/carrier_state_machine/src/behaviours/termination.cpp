#include <carrier_state_machine/behaviours/termination.hpp>

namespace ariitk::carrier_state_machine {

void Termination::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
}

void Termination::execute(const Event& evt) {
}

}  // namespace ariitk::carrier_state_machine
