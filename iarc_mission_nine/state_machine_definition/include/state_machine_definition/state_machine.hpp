#pragma once

#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <cxxabi.h>
#include <ros/ros.h>

#define FSM_INFO(X) ROS_WARN_STREAM("[FSM]: " << X)

namespace ariitk::state_machine {

// store available policies for easier use
struct SwitchPolicy {
    typedef boost::msm::active_state_switch_after_entry after_entry;
    typedef boost::msm::active_state_switch_after_exit after_exit;
    typedef boost::msm::active_state_switch_before_transition before_transition;
    typedef boost::msm::active_state_switch_after_transition_action after_transition;
};

template<class FSMClass>
class FSMDef : public boost::msm::front::state_machine_def<FSMClass> {
  public:
    typedef boost::msm::back::state_machine<FSMClass> StateMachineBackend;
    typedef SwitchPolicy::after_transition active_state_switch_policy;

    template<class Event, class FSM>
    void on_entry(Event const&, FSM&){};
    template<class Event, class FSM>
    void on_exit(Event const&, FSM&){};

    template<class T, bool V = false>
    struct State : public boost::msm::front::state<> {
        State() {
            state_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, nullptr);
            verbose = V;
        }

        template<class Event, class FSM>
        void on_entry(Event const&, FSM&) {
            if (verbose) {
                FSM_INFO("Entered " << state_name << " state");
            }
        }

        template<class Event, class FSM>
        void on_exit(Event const&, FSM&) {
            if (verbose) {
                FSM_INFO("Exited " << state_name << " state");
            }
        }

        std::string state_name;
        bool verbose;
    };
};

}  // namespace ariitk::state_machine
