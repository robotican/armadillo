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

#ifndef ARMADILLO2_HW_ARMADILLO_HW_H
#define ARMADILLO2_HW_ARMADILLO_HW_H

#include "dxl_motors_builder.h"
#include "battery_pub.h"
#include "roboteq_diff_drive.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#define TORSO_JOINT_PARAM "~torso_joint"

namespace armadillo2_hw
{

    struct torso_joint
    {
        double pos = 0;
        double vel = 0;
        double prev_pos = 0;
        double effort = 0; /* effort stub - not used */
        double command_effort = 0;
        std::string joint_name;
    };


    class ArmadilloHW : public hardware_interface::RobotHW
    {
    private:

        /* handles */
        std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
        std::vector<hardware_interface::JointHandle> pos_handles_;
        torso_joint torso_;

        ros::Publisher servo_pub_;
        ros::Subscriber servo_sub_;
        ros::Subscriber emergency_sub_;

        ros::Time prev_time_;
        ros::Time servo_pub_prev_time_,servo_sub_prev_time_;

        ros::NodeHandle *node_handle_;

        /* interfaces */
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_interface_;
        hardware_interface::PosVelJointInterface posvel_interface_;
        hardware_interface::VelocityJointInterface velocity_interface_;
        hardware_interface::EffortJointInterface effort_interface_;

        /* robot close loop components */
        DxlMotorsBuilder dxl_motors_;
        BatteryPub battery_;
        RoboteqDiffDrive roboteq_;

        ros::Publisher espeak_pub_;

        void torsoCallback(const std_msgs::Float32::ConstPtr& msg);
        void emergencyCallback(const std_msgs::Bool::ConstPtr &msg);

        void registerInterfaces();
        void straighHead();
        void speakMsg(std::string msg, int sleep_time)
        {
            std_msgs::String speak_msg;
            speak_msg.data = msg;
            espeak_pub_.publish(speak_msg);
            if (sleep_time > 0)
                sleep(sleep_time);
        }

    public:

        ArmadilloHW(ros::NodeHandle &nh);
        void read();
        void write();
    };
}

#endif //ARMADILLO2_HW_ARMADILLO_HW_H
