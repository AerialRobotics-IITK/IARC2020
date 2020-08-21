#pragma once

#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include <cxxabi.h>
#include <ros/ros.h>

#define echo(X) ROS_WARN_STREAM("[FSM]: " << X)  // TODO: change name of macro

namespace ariitk::state_machine {

template<class FSMClass>
class FSMDef : public boost::msm::front::state_machine_def<FSMClass> {
  public:
    typedef boost::msm::back::state_machine<FSMClass> StateMachineBackend;
    typedef boost::msm::active_state_switch_after_transition_action active_state_switch_policy;

    template<class Event, class FSM>
    void on_entry(Event const&, FSM&){};
    template<class Event, class FSM>
    void on_exit(Event const&, FSM&){};

    template<class T, bool V = false>
    struct State : public boost::msm::front::state<> {
        State() {
            int status;                                                         // TODO: replace with placeholder
            state_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, &status);  // TODO: Discard namespacing
            verbose = V;
        }

        template<class Event, class FSM>
        void on_entry(Event const&, FSM&) {
            if (verbose) {
                echo("Entered " << state_name << " state");
            }
        }

        template<class Event, class FSM>
        void on_exit(Event const&, FSM&) {
            if (verbose) {
                echo("Exited " << state_name << " state");
            }
        }

        char* state_name;  // TODO: replace with string
        bool verbose;
    };

    struct Command {
        Command(){};
    };
};

}  // namespace ariitk::state_machine
