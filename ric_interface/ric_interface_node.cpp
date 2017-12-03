#include <ros/ros.h>
#define LOOP_HZ 100.0
#define THREADS_NUM 2

#include <ric_interface/ric_interface.h>

ric_interface::RicInterface bm;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ric_interface_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    while (ros::ok())
    {
        bm.loop();
        ros::Duration((1000.0 / LOOP_HZ)/1000.0).sleep();
        ros::spinOnce;
    }
}




