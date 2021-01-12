#pragma once

#include <agent_state_machine/agent_state/agent_state.hpp>
#include <mast_finder/mast_finder.hpp>
#include <plate_detector/plate_detector.hpp>
#include <pose_estimator/pose_estimator.hpp>
#include <detector_msgs/Centre.h>

namespace ariitk::agent_state_machine {

class MastSearch {
  public:
    struct Event {};

    void execute(const Event evt);
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::shared_ptr<AgentState> state_ptr);

  private:
    std::shared_ptr<AgentState> mav_state_;  // TODO: Create Destructors
    ariitk::plate_detection::PlateDetectorNode plate_detector_;
    ariitk::pose_estimation::PoseEstimatorNode pose_estimator_;
    ariitk::mast_locator::MastLocatorNode mast_finder_;
};

}  // namespace ariitk::agent_state_machine