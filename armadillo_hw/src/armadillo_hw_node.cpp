
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "armadillo_hw.h"

#define LOOP_INTERVAL 100
#define THREADS_NUM 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armadillo2_hw_node");
    ros::NodeHandle nh;

    ArmadilloHW armadillo_hw(nh);
    controller_manager::ControllerManager controller_manager(&armadillo_hw);

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    ros::Rate loop(LOOP_INTERVAL);
    while (ros::ok())
    {
        armadillo_hw.read();
        controller_manager.update(armadillo_hw.getTime(), armadillo_hw.getPeriod());
        armadillo_hw.write();

        loop.sleep();
    }
}