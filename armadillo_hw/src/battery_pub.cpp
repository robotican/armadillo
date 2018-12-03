/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elchay Rauper*/


#include "battery_pub.h"

namespace armadillo2_hw
{
    BatteryPub::BatteryPub(ros::NodeHandle nh)
    {
        /* get batt params */
        ros::param::get("~load_battery_hw", load_battery_hw_);

        if (!load_battery_hw_)
        {
            ROS_WARN("[armadillo2_hw/battery_pub]: battery hardware is disabled");
            return;
        }

        if (!ros::param::get(BATT_PORT_PARAM, batt_port_))
        {
            ROS_ERROR("[armadillo2_hw/battery_pub]: %s param is missing on param server. make sure that this param exist in battery_config.yaml "
                              "and that your launch includes this param file. shutting down...", BATT_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }
        ros::param::get("~low_batt_val", low_batt_val_);
        ros::param::get("~show_warnings", show_warnings_);

        /* connect to batt FTDI */
        try
        {
            bms_.connect(batt_port_);
        }
        catch (bms::BMSException exp)
        {
            ROS_ERROR("[armadillo2_hw/battery_pub]: %s", exp.what());
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
        ROS_INFO("[armadillo2_hw/battery_pub]: battery port opened successfully \nport name: %s \nbaudrate: 9600", batt_port_.c_str());

        /* batt publisher */
        bat_pub_ = nh.advertise<sensor_msgs::BatteryState>("battery", 10);
        bat_pub_timer_ = nh.createTimer(ros::Duration(BATT_PUB_INTERVAL), &BatteryPub::pubBatTimerCB, this);
        speak_low_batt_timer_ = nh.createTimer(ros::Duration(SPEAK_LOW_BAT_INTERVAL), &BatteryPub::speakLowTimerCB, this);
        speak_low_batt_timer_.stop();
        ROS_INFO("[armadillo2_hw/battery_pub]: battery publisher is up");
        espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        /*speakMsg("battery management system is up",1);*/
    }

    void BatteryPub::speakLowTimerCB(const ros::TimerEvent &event)
    {
        speakMsg("low battery", 0);
    }

    void BatteryPub::pubBatTimerCB(const ros::TimerEvent &event)
    {
        try
        {
            bms::data bms_data =  bms_.read();

            sensor_msgs::BatteryState msg;
            msg.header.stamp = ros::Time::now();
            msg.present = true;
            msg.voltage = bms_data.vbat;
            msg.percentage = bms_data.soc;
            msg.current = bms_data.chrg_current - bms_data.dchrg_current;
            msg.charge = bms_data.chrg_current;
            msg.capacity = bms_data.cap_full; //Ah
            msg.power_supply_status = bms_data.is_chrg;
            msg.cell_voltage = bms_data.vcells;
            msg.location = "base_link";

            /* if battery low and not in charging print warning */
            if ((low_batt_val_ >=0 && msg.percentage <= low_batt_val_) && !bms_data.is_chrg)
            {
                ROS_WARN("[armadillo2_hw/battery_pub]: LOW BATTERY, please connect Armadillo2 to charger");
                speak_low_batt_timer_.start();
            }
            else
                speak_low_batt_timer_.stop();

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
            if (show_warnings_)
                ROS_WARN("[armadillo2_hw/battery_pub]: %s", exp.what());
        }
    }
}
