#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armadillo2_teleop/armadillo_teleop.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armadillo2_teleop_node");
    ros::NodeHandle nh;
    /* must use multi threaded spinner for moveit */
    ros::AsyncSpinner spinner(2);
    spinner.start();

    Armadillo2Teleop armadillo_teleop;
    ros::waitForShutdown();
    return 0;
}

