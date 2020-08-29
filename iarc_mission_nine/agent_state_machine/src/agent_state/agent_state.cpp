#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {

void AgentState::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("call_rate", call_rate_);
    nh_private.getParam("distance_error", distance_error_);

    odom_sub_ = nh.subscribe("odometry", 1, &AgentState::odometryCallback, this);
    state_sub_ = nh.subscribe("mavros/state", 1, &AgentState::stateCallback, this);

    mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    pose_.position.z = DBL_MIN;
    ros::Rate loop_rate(2);

    // wait for first message
    while (pose_.position.z == DBL_MIN) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void AgentState::odometryCallback(const nav_msgs::Odometry& odom) {
    pose_ = odom.pose.pose;
}

void AgentState::stateCallback(const mavros_msgs::State& state) {
    state_ = state;
}

// TODO: replace string with an enum table
bool AgentState::switchMode(const std::string& des_mode) {
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

void AgentState::goToLocation(const geometry_msgs::Pose& des_pose) {
    mavros_msgs::SetMode mode_srv;
    mode_srv.request.custom_mode = "OFFBOARD";

    geometry_msgs::PoseStamped setpt_msg;
    setpt_msg.pose = des_pose;

    ros::Rate loop_rate(call_rate_);
    uint attempts = 0, max_attempts = 500;

    while (ros::ok()) {
        ros::spinOnce();
        setpt_msg.header.stamp = ros::Time::now();
        cmd_pose_pub_.publish(setpt_msg);

        ROS_INFO_STREAM(pointDistance(pose_.position, des_pose.position) << " " << distance_error_);

        if (state_.mode != "OFFBOARD") {
            mode_client_.call(mode_srv);
            if (++attempts > max_attempts) {
                ROS_ERROR_STREAM("Offboard Switch unsuccessful!");
                break;
            }
        } else if (pointDistance(pose_.position, des_pose.position) <= distance_error_) {
            switchMode("AUTO.LOITER");
            break;
        }

        loop_rate.sleep();
    }
}

}  // namespace ariitk::agent_state_machine
