#include <carrier_state_machine/behaviours/reach_ship.hpp>

namespace ariitk::carrier_state_machine {

void ReachShip::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
    ROS_ERROR("reachship init called");
}

void ReachShip::execute(const Event& evt) {
    ROS_ERROR("reachship execute called");
    rosplane::path_manager_example* it = new rosplane::path_manager_example();
    it->forwardModeOn();  // It makes the boolean forward_ true which is to be used in path_manager_example.cpp to call forwardRun().

    rosplane::path_manager_base* est = new rosplane::path_manager_example();  // Assigned the pointer of path_manager_example to the pointer of
                                                                              // path_manager_base as the former is a class inherited from the latter one.
    est->forwardRun();
    ros::spin();
    ROS_ERROR("reachship execute successfully executed");
}

}  // namespace ariitk::carrier_state_machine
