cmake_minimum_required(VERSION 2.8.3)
project(armadillo2_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    std_srvs
    trajectory_msgs
    geometry_msgs
    message_generation
    actionlib
)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)


add_message_files(
    DIRECTORY msgs
    FILES
    JointTrajectoryControllerState.msg
)

## Generate services in the 'srv' folder
 add_service_files(
    DIRECTORY srvs
    FILES
    PanTilt.srv
    QueryTrajectoryState.srv
    SwitchCamTopic.srv
 )

add_action_files(
   DIRECTORY action
   FILES
   PointHead.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   trajectory_msgs 
   std_srvs
   geometry_msgs
   actionlib_msgs
 )

catkin_package(
    CATKIN_DEPENDS 
    actionlib_msgs 
    trajectory_msgs 
    geometry_msgs 
    message_runtime
)


catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime
)
