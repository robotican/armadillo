
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "armadillo2_hw.h"

#define LOOP_HZ 100
#define THREADS_NUM 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armadillo2_hw_node");
    ros::NodeHandle nh;

    armadillo2_hw::ArmadilloHW armadillo_hw(nh);
    controller_manager::ControllerManager controller_manager(&armadillo_hw);

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    ros::Rate loop(LOOP_HZ);
    while (ros::ok())
    {
        armadillo_hw.read();
        controller_manager.update(armadillo_hw.getTime(), armadillo_hw.getPeriod());
        //armadillo_hw.write();

        ros::spinOnce;
        loop.sleep();
    }
}