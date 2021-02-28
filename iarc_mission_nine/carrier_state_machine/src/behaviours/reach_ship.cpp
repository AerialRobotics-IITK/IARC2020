#include <carrier_state_machine/behaviours/reach_ship.hpp>

namespace ariitk::carrier_state_machine {

void ReachShip::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
    ROS_ERROR("reachship init called");

    ROS_ERROR("reachship init executed completely");
}

void ReachShip::execute(const Event& evt) {
    rosplane::path_manager_base* est = new rosplane::path_manager_example();
    est->forwardRun();
    // ros::Rate looprate(10);
    // while (ros::ok()) {
    //     looprate.sleep();

    //     ros::spinOnce();
    // }
    ros::spin();
}

}  // namespace ariitk::carrier_state_machine
