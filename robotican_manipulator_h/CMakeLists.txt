cmake_minimum_required(VERSION 2.8.3)
project(robotican_manipulator_h)

find_package(catkin REQUIRED COMPONENTS
        armadillo_hw
        std_msgs
        actionlib
        control_msgs
        control_toolbox
        hardware_interface
        controller_manager
        serial
        moveit_core
        pluginlib
        roscpp
        tf_conversions
        )

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
        LIBRARIES
        CATKIN_DEPENDS
        moveit_core
        pluginlib
        roscpp
        tf_conversions
        armadillo_hw
        std_msgs
        actionlib
        control_msgs
        control_toolbox
        hardware_interface
        controller_manager
        serial
)

include_directories(include)

set(IKFAST_LIBRARY_NAME robotican_manipulator_h_arm_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME} src/robotican_manipulator_h_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

#add_executable(arm_node src/arm_node.cpp)
#target_link_libraries(arm_node ${catkin_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
        FILES
        robotican_manipulator_h_arm_moveit_ikfast_plugin_description.xml
        DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
