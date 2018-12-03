cmake_minimum_required(VERSION 2.8.3)
project(armadillo1_services)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        armadillo1_msgs
        roscpp
#        std_msgs
        message_generation
        genmsg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES armadillo1_msgs
  CATKIN_DEPENDS
        message_runtime
        roscpp
        armadillo1_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/armadillo1_services_node.cpp
        src/dxl_torque.cpp
        src/lift_arm.cpp
        src/pan_tilt_mover.cpp
        src/shutdown.cpp
        include/armadillo1_services/dxl_torque.h
        include/armadillo1_services/lift_arm.h
        include/armadillo1_services/pan_tilt_mover.h
        include/armadillo1_services/shutdown.h src/torso.cpp include/armadillo1_services/torso.h src/joints_state_reader.cpp include/armadillo1_services/joints_state_reader.h)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node armadillo1_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )