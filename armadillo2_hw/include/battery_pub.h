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


#ifndef ARMADILLO2_HW_BATTERY_PUB_H
#define ARMADILLO2_HW_BATTERY_PUB_H

#include <ros/ros.h>
#include <bms_interface/bms_interface.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

//#define BATT_PORT "/dev/BMS"
#define BATT_PUB_INTERVAL 2 //secs
#define SPEAK_LOW_BAT_INTERVAL 30 //secs
#define BATT_PORT_PARAM "~batt_port"

namespace armadillo2_hw
{
    class BatteryPub
    {
    private:
        ros::Publisher bat_pub_;
        ros::Timer bat_pub_timer_;
        ros::Timer speak_low_batt_timer_;
        bms::BMSInterface bms_;

        std::string batt_port_;
        int low_batt_val_ = -1;
        bool show_warnings_ = false;
        bool load_battery_hw_ = true;

        ros::Publisher espeak_pub_;

        void pubBatTimerCB(const ros::TimerEvent &event);

        void speakLowTimerCB(const ros::TimerEvent& event);

        void speakMsg(std::string msg, int sleep_time)
        {
            std_msgs::String speak_msg;
            speak_msg.data = msg;
            espeak_pub_.publish(speak_msg);
            if (sleep_time > 0)
                sleep(sleep_time);
        }

    public:
        BatteryPub(ros::NodeHandle nh);
    };
}



#endif //ARMADILLO2_HW_BATTERY_PUB_H
