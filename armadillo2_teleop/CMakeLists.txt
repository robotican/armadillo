cmake_minimum_required(VERSION 2.8.3)
project(armadillo2_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    joy
    roscpp
    actionlib
    moveit_ros_planning_interface
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES armadillo2_teleop
  CATKIN_DEPENDS joy roscpp actionlib
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/armadillo2_teleop_node.cpp
        include/armadillo2_teleop/armadillo_teleop.h
        src/armadillo_teleop.cpp
        include/armadillo2_teleop/joy_profile.h)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )