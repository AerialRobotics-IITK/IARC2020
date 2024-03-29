cmake_minimum_required(VERSION 2.8.3)
project(agent_state_machine)

find_package(catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(agent_state src/agent_state/agent_state.cpp)
target_link_libraries(agent_state ${catkin_LIBRARIES})

cs_add_library(agent_behaviours 
    src/behaviours/attach_block.cpp
    src/behaviours/detach_block.cpp
    src/behaviours/hovering.cpp
    src/behaviours/initialization.cpp
    src/behaviours/mast_search.cpp
    src/behaviours/termination.cpp
)
target_link_libraries(agent_behaviours agent_state ${catkin_LIBRARIES})

cs_add_library(${PROJECT_NAME}_lib 
    src/agent_state_machine.cpp
    src/agent_state_machine_base.cpp
)
target_link_libraries(${PROJECT_NAME}_lib agent_state agent_behaviours)

cs_add_executable(${PROJECT_NAME}_node 
    src/agent_state_machine_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

cs_install()
cs_export()
