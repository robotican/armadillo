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


#ifndef ARMADILLO2_HW_ROBOTEQ_DIFF_DRIVE_H
#define ARMADILLO2_HW_ROBOTEQ_DIFF_DRIVE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <roboteq/roboteq.h>
#include <roboteq/serial_controller.h>
#include <std_msgs/String.h>

#define ROBOTEQ_PORT_PARAM "~roboteq_port"
#define ROBOTEQ_BAUD_PARAM "~roboteq_baud"
#define RIGHT_WHEEL_JOINT_PARAM "~right_wheel_joint"
#define LEFT_WHEEL_JOINT_PARAM "~left_wheel_joint"

typedef boost::chrono::steady_clock time_source;

class RoboteqDiffDrive
{
private:

    ros::NodeHandle *nh_;
    roboteq::serial_controller *roboteq_serial_;
    roboteq::Roboteq *roboteq_;
    time_source::time_point last_time_ = time_source::now();

    std::string roboteq_port_;
    std::string right_wheel_joint_, left_wheel_joint_;
    int roboteq_baud_ = 0;
    bool load_roboteq_hw_ = false;
    /* if first time, subtract previous values */
    bool first_time_ = true;
    ros::Publisher espeak_pub_;

    void speakMsg(std::string msg, int sleep_time)
    {
        std_msgs::String speak_msg;
        speak_msg.data = msg;
        espeak_pub_.publish(speak_msg);
        if (sleep_time > 0)
            sleep(sleep_time);
    }


public:
    ~RoboteqDiffDrive() { delete roboteq_; }
    RoboteqDiffDrive(ros::NodeHandle &nh);
    void read(const ros::Duration elapsed);
    void write(const ros::Duration elapsed);
    void registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                         hardware_interface::VelocityJointInterface &velocity_joint_interface);
};


#endif //ARMADILLO2_HW_ROBOTEQ_DIFF_DRIVE_H
