//
// Created by sub on 23/10/17.
//

#include <ros/ros.h>

#include "bms/bms_interface.h"

#define LOOP_INTERVAL 1000
#define THREADS_NUM 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ip_estimator_node");
    ros::NodeHandle nh;

    bms::BMSInterface bms;
    try
    {
        bms.connect("/dev/ttyUSB0");
    }
    catch (bms::BMSException exp)
    {
        ROS_ERROR("[bms_test]: %s", exp.what());
        //ros::shutdown();
    }

    ros::Rate loop_rate(1);


    while (ros::ok())
    {

        try
        {
            bms.read();
        }
        catch(bms::BMSErrorException exp)
        {
            ROS_ERROR("[bms_test]: %s", exp.what());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        catch(bms::BMSWarnException exp)
        {
            ROS_WARN("[bms_test]: %s", exp.what());
        }


        loop_rate.sleep();
        ros::spinOnce();
    }

}