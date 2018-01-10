cmake_minimum_required(VERSION 2.8.3)
project(armadillo2_hw)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    controller_manager
    bms_interface
    dxl_interface
    ric_interface
    roboteq_control
    filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES armadillo2_hw
  CATKIN_DEPENDS bms_interface
        dxl_interface
        ric_interface
        roboteq_control
        filters
DEPENDS
    Boost
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node
                src/armadillo2_hw.cpp
                include/armadillo2_hw.h
         src/armadillo2_hw_node.cpp
         src/dxl_motors_builder.cpp
         include/dxl_motors_builder.h
         src/battery_pub.cpp
         include/battery_pub.h
         src/ricboard_pub.cpp
         include/ricboard_pub.h
         src/roboteq_diff_drive.cpp
         include/roboteq_diff_drive.h)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(
         ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
 )