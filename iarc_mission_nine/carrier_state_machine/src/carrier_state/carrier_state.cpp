#include <carrier_state_machine/carrier_state/carrier_state.hpp>

namespace ariitk::carrier_state_machine {

void CarrierState::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("call_rate", call_rate_);

    odom_sub_ = nh.subscribe("odometry", 1, &CarrierState::odometryCallback, this);
    state_sub_ = nh.subscribe("mavros/state", 1, &CarrierState::stateCallback, this);

    mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    pose_.position.z = DBL_MIN;
    ros::Rate loop_rate(2);

    // wait for first message
    while (pose_.position.z == DBL_MIN) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CarrierState::odometryCallback(const nav_msgs::Odometry& odom) {
    pose_ = odom.pose.pose;
}

void CarrierState::stateCallback(const mavros_msgs::State& state) {
    state_ = state;
}

// TODO: replace string with an enum table
bool CarrierState::switchMode(const std::string& des_mode) {
    mavros_msgs::SetMode mode_srv;
    mode_srv.request.custom_mode = des_mode;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 69;

    // TODO: Try offboard instead of takeoff service
    while (ros::ok() && state_.mode != des_mode) {
        mode_client_.call(mode_srv);
        ros::spinOnce();

        if (attempts++ > max_attempts) {
            ROS_ERROR_STREAM("Switch unsuccessful!");
            return false;
        }

        loop_rate.sleep();
    }

    return true;
}

}  // namespace ariitk::carrier_state_machine
