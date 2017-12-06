//
// Created by sub on 31/10/17.
//

#ifndef ARMADILLO2_HW_BATTERY_PUB_H
#define ARMADILLO2_HW_BATTERY_PUB_H

#include <ros/ros.h>
#include <bms_interface/bms_interface.h>
#include <sensor_msgs/BatteryState.h>

//#define BATT_PORT "/dev/BMS"
#define BATT_PUB_INTERVAL 1 //secs
#define BATT_PORT_PARAM "batt_port"

namespace armadillo2_hw
{
    class BatteryPub
    {
    private:
        ros::Publisher bat_pub_;
        ros::Timer bat_pub_timer_;
        bms::BMSInterface bms_;

        std::string batt_port_;
        int low_batt_val_ = -1;
        bool show_warnings_ = false;
        bool load_battery_hw_ = true;



        void pubTimerCB(const ros::TimerEvent& event);

    public:
        BatteryPub(ros::NodeHandle nh);
    };
}



#endif //ARMADILLO2_HW_BATTERY_PUB_H
