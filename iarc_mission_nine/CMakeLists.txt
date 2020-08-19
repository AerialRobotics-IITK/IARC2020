cmake_minimum_required(VERSION 2.8.3)
project(iarc_mission_nine)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

set(ASM agent_state_machine)
set(CSM carrier_state_machine)

cs_add_library(${ASM}_lib
    src/${ASM}/${ASM}_base.cpp
    src/${ASM}/${ASM}.cpp
)
target_link_libraries(${ASM}_lib ${catkin_LIBRARIES})

cs_add_executable(${ASM}_node src/${ASM}/${ASM}_node.cpp)
target_link_libraries(${ASM}_node ${ASM}_lib)

cs_add_library(${CSM}_lib
    src/${CSM}/${CSM}_base.cpp
    src/${CSM}/${CSM}.cpp
)
target_link_libraries(${CSM}_lib ${catkin_LIBRARIES})

cs_add_executable(${CSM}_node src/${CSM}/${CSM}_node.cpp)
target_link_libraries(${CSM}_node ${CSM}_lib)
