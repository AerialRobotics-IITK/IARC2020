cmake_minimum_required(VERSION 2.8.3)
project(agent_state_machine)

set ( DEPENDENCIES 
    nav_msgs
    geometry_msgs
    mavros_msgs
    state_machine_definition
    mast_finder
    plate_detector
    led_detector
    feature_detection
    detector_msgs
    pose_estimator
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    ${DEPENDENCIES}
)

add_definitions(-std=c++17 -g)
set(CMAKE_BUILD_TYPE Release)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES agent_state agent_behaviours agent_state_machine_lib
    CATKIN_DEPENDS ${DEPENDENCIES}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(agent_state
    src/agent_state/agent_state.cpp
)
target_link_libraries(agent_state ${catkin_LIBRARIES})

add_library(agent_behaviours
    src/behaviours/initialization.cpp
    src/behaviours/hovering.cpp
    src/behaviours/termination.cpp
    src/behaviours/mast_search.cpp
    src/behaviours/attach_block.cpp
    src/behaviours/detach_block.cpp
)
target_link_libraries(agent_behaviours agent_state)

add_library(agent_state_machine_lib
    src/agent_state_machine_base.cpp
    src/agent_state_machine.cpp
)
target_link_libraries(agent_state_machine_lib agent_behaviours)

add_executable(agent_state_machine_node src/agent_state_machine_node.cpp)
target_link_libraries(agent_state_machine_node agent_state_machine_lib)

install(TARGETS agent_state agent_state_machine_lib agent_behaviours
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS agent_state_machine_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)