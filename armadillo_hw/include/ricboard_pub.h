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

#ifndef ARMADILLO_HW_RICBOARD_PUB_H
#define ARMADILLO_HW_RICBOARD_PUB_H

#include <ric_interface/ric_interface.h>
#include <ric_interface/ric_exception.h>
#include <filters/low_pass_filter.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono/chrono.hpp>


#define RIC_PORT_PARAM "~ric_port"
#define TORSO_JOINT_PARAM "~torso_joint"
#define RIC_PUB_INTERVAL 0.1 //secs
#define RIC_WRITE_INTERVAL 0.05 //secs
#define RIC_DEAD_TIMEOUT 1 //secs
#define MAX_RIC_DISCONNECTIONS 5
#define SERVO_NEUTRAL 1500
#define G_FORCE 9.80665


struct torso_joint
{
    double pos = 0;
    double vel = 0;
    double prev_pos = 0;
    double effort = 0; /* effort stub - not used */
    double command_effort = 0;
    std::string joint_name;
};


class RicboardPub
{
private:

    bool  load_ric_hw_ = true;
    int ric_disconnections_counter_ = 0;
    std::string ric_port_;
    ros::Publisher ric_gps_pub_;
    ros::Publisher ric_ultrasonic_pub_;
    ros::Publisher ric_imu_pub_;
    ros::Publisher ric_mag_pub_;

    ros::Timer ric_pub_timer_,
               ric_dead_timer_;
    torso_joint torso_;
    //LowPassFilter torso_lpf_;
    ric::RicInterface ric_;
    ros::NodeHandle *nh_;
    boost::thread* t;

    ros::Publisher espeak_pub_;

    /* handles */
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
    std::vector<hardware_interface::JointHandle> pos_handles_;

    void pubTimerCB(const ros::TimerEvent& event);
    void ricDeadTimerCB(const ros::TimerEvent& event);
    void loop();
    void speakMsg(std::string msg, int sleep_time)
    {
        std_msgs::String speak_msg;
        speak_msg.data = msg;
        espeak_pub_.publish(speak_msg);
        if (sleep_time > 0)
            sleep(sleep_time);
    }

public:
    RicboardPub(ros::NodeHandle &nh);
    void startLoop();
    void stopLoop();

    /* functions for ros controller use */
    void read(const ros::Duration elapsed);
    void write(const ros::Duration elapsed);
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::EffortJointInterface &position_interface);
};


#endif //ARMADILLO_HW_RICBOARD_PUB_H
