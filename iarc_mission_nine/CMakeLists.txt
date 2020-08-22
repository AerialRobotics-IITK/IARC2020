cmake_minimum_required(VERSION 2.8.3)
project(iarc_mission_nine)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(agent_behaviours
    src/agent_transition_behaviours/initialization.cpp
)
target_link_libraries(agent_behaviours ${catkin_LIBRARIES})

cs_add_library(agent_state_machine_lib
    src/agent_state_machine/agent_state_machine_base.cpp
    src/agent_state_machine/agent_state_machine.cpp
)
target_link_libraries(agent_state_machine_lib agent_behaviours)

cs_add_executable(agent_state_machine_node src/agent_state_machine/agent_state_machine_node.cpp)
target_link_libraries(agent_state_machine_node agent_state_machine_lib)

cs_add_library(carrier_state_machine_lib
    src/carrier_state_machine/carrier_state_machine_base.cpp
    src/carrier_state_machine/carrier_state_machine.cpp
)
target_link_libraries(carrier_state_machine_lib ${catkin_LIBRARIES})

cs_add_executable(carrier_state_machine_node src/carrier_state_machine/carrier_state_machine_node.cpp)
target_link_libraries(carrier_state_machine_node carrier_state_machine_lib)
