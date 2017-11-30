//
// Created by sub on 31/10/17.
//

#include "battery_pub.h"

namespace armadillo2_hw
{
    BatteryPub::BatteryPub(ros::NodeHandle nh)
    {
        try
        {
            bms_.connect(BATT_PORT);
        }
        catch (bms::BMSException exp)
        {
            ROS_ERROR("[armadillo2_hw/battery_pub]: %s", exp.what());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        ROS_INFO("[armadillo2_hw/battery_pub]: battery port opened successfully \nport name: %s \nbaudrate: 9600", BATT_PORT);

        bat_pub_ = nh.advertise<sensor_msgs::BatteryState>("battery_state", 10);
        bat_pub_timer_ = nh.createTimer(ros::Duration(BATT_PUB_INTERVAL), &BatteryPub::pubTimerCB, this);
        ROS_INFO("[armadillo2_hw/battery_pub]: battery publisher is up");
    }

    void BatteryPub::pubTimerCB(const ros::TimerEvent &event)
    {

        try
        {
            bms::data bms_data =  bms_.read();

            sensor_msgs::BatteryState msg;
            msg.present = true;
            msg.voltage = bms_data.vbat;
            msg.percentage = bms_data.soc;
            msg.current = bms_data.chrg_current - bms_data.dchrg_current;
            msg.charge = bms_data.chrg_current;
            msg.capacity = bms_data.cap_full; //Ah
            msg.power_supply_status = bms_data.is_chrg;
            msg.cell_voltage = bms_data.vcells;
            msg.location = "base_link";

            bat_pub_.publish(msg);
        }
        catch(bms::BMSErrorException exp)
        {
            ROS_ERROR("[armadillo2_hw/battery_pub]: %s", exp.what());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        catch(bms::BMSWarnException exp)
        {
            if (SHOW_WARNINGS)
                ROS_WARN("[armadillo2_hw/battery_pub]: %s", exp.what());
        }


        //TODO: PUBLISH DATA
    }

    void BatteryPub::startPublish()
    {
        bat_pub_timer_.start();
    }

    void BatteryPub::stopPublish()
    {
        bat_pub_timer_.stop();
    }
}
