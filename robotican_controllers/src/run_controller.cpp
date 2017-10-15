//
// Created by tom on 03/04/16.
//

#include <ros/ros.h>
#include <robotican_controllers/robot.h>
#include <controller_manager/controller_manager.h>
//#include <robotican_controllers/two_finger_controller.h>
//#include <robotican_controllers/hardware_interface_adapter.h>
#include <robotican_controllers/joint_trajectory_controller.h>

int main(int argc, char** argv) {
    ros::init(argc, argv,"robotican_controllers");
    RobotArm robotArm;
    controller_manager::ControllerManager controllerManager(&robotArm);
    ros::Rate rate(50);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok()) {
        controllerManager.update(robotArm.getTime(), robotArm.getPeriod());
        rate.sleep();
    }
    return 0;
}
