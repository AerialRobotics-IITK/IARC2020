#include <carrier_state_machine/carrier_state_machine.hpp>

using namespace ariitk::carrier_state_machine;

int main(int argc, char** argv) {
    ros::init(argc, argv, "carrier_state_machine");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    CarrierStateMachine fsm(nh, nh_private);
    fsm.spin();
}
