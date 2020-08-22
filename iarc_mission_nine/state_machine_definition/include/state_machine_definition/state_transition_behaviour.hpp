#pragma once

#include <ros/ros.h>

#define BHV_INFO(X) ROS_INFO_STREAM("[BHV]: " << X)

namespace ariitk::state_machine {

class Behaviour {
  public:
    struct Event {};

    virtual bool guard(const Event& evt) {
        // allow transitions by default
        return true;
    }

    virtual void action(const Event& evt){};
};

}  // namespace ariitk::state_machine
