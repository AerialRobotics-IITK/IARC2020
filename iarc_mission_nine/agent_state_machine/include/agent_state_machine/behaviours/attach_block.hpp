#pragma once

#include <agent_state_machine/agent_state/agent_state.hpp>
#include "gazebo_ros_link_attacher/Attach.h"          //! Paths are mp incorrect
#include "gazebo_ros_link_attacher/AttachRequest.h"
#include "gazebo_ros_link_attacher/AttachResponse.h"

namespace ariitk::agent_state_machine {

class AttachBlock {
  public:
    struct Event {};

    void execute(const Event evt);
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    std::shared_ptr<AgentState> mav_state_;
    ros::ServiceClient attach_client_;
    gazebo_ros_link_attacher::AttachRequest req_;    //! Be wary of Namespace
};

}  // namespace ariitk::agent_state_machine