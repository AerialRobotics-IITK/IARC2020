#pragma once

<<<<<<< HEAD
=======
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

>>>>>>> a3d2e67753f208637e1166982431f4a5d133d9e4
#include <agent_state_machine/agent_state/agent_state.hpp>

namespace ariitk::agent_state_machine {

class Initialization {
  public:
    struct Event {};

    void execute(const Event& evt);
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    bool takeoff();

    double call_rate_;
    double hover_height_;
    double distance_error_;

    std::shared_ptr<AgentState> mav_state_;
};

<<<<<<< HEAD
}  // namespace ariitk::agent_state_machine
=======
}  // namespace ariitk::agent_state_machine
>>>>>>> a3d2e67753f208637e1166982431f4a5d133d9e4
