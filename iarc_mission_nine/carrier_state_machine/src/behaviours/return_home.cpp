#include <carrier_state_machine/behaviours/return_home.hpp>

namespace ariitk::carrier_state_machine {

void ReturnHome::init(ros::NodeHandle nh, ros::NodeHandle nh_private, const std::shared_ptr<CarrierState> state_ptr) {
    ROS_ERROR("returnhome init called");
}

void ReturnHome::execute(const Event& evt) {
    ROS_ERROR("returnhome execute called");
    rosplane::path_manager_example* it = new rosplane::path_manager_example();
    it->backwardModeOn();  // It makes the boolean forward_ false which is to be used in path_manager_example.cpp to call backwardRun().

    rosplane::path_manager_base* est = new rosplane::path_manager_example();  // Assigned the pointer of path_manager_example to the pointer of
                                                                              // path_manager_base as the former is a class inherited from the latter one.
    est->backwardRun();
    ros::spin();
    ROS_ERROR("returnhome execute successfully executed");
}

}  // namespace ariitk::carrier_state_machine
