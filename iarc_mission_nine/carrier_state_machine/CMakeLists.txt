cmake_minimum_required(VERSION 2.8.3)
project(carrier_state_machine)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(carrier_state
    src/carrier_state/carrier_state.cpp
)
target_link_libraries(carrier_state ${catkin_LIBRARIES})

cs_add_library(carrier_behaviours
    src/behaviours/hovering.cpp
    src/behaviours/termination.cpp
    src/behaviours/deploy_agent.cpp
    src/behaviours/reach_ship.cpp
    src/behaviours/return_home.cpp
    src/behaviours/takeoff.cpp
)
target_link_libraries(carrier_behaviours carrier_state)

cs_add_library(carrier_state_machine_lib
    src/carrier_state_machine_base.cpp
    src/carrier_state_machine.cpp
)
target_link_libraries(carrier_state_machine_lib carrier_behaviours)

cs_add_executable(carrier_state_machine_node src/carrier_state_machine_node.cpp)
target_link_libraries(carrier_state_machine_node carrier_state_machine_lib)

cs_install()
cs_export()
