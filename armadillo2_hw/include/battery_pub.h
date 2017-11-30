//
// Created by sub on 31/10/17.
//

#ifndef ARMADILLO2_HW_BATTERY_PUB_H
#define ARMADILLO2_HW_BATTERY_PUB_H

#include <ros/ros.h>
#include <bms/bms_interface.h>
#include <sensor_msgs/BatteryState.h>

#define BATT_PORT "/dev/BMS"
#define BATT_PUB_INTERVAL 1 //secs
#define SHOW_WARNINGS false

namespace armadillo2_hw
{
    class BatteryPub
    {
    private:
        ros::NodeHandle *node_handle_;
        ros::Publisher bat_pub_;
        ros::Timer bat_pub_timer_;
        bms::BMSInterface bms_;

        void pubTimerCB(const ros::TimerEvent& event);

    public:
        BatteryPub(ros::NodeHandle nh);
        void startPublish();
        void stopPublish();
    };
}



#endif //ARMADILLO2_HW_BATTERY_PUB_H
