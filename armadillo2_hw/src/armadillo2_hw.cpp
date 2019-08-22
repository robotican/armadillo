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

#include "armadillo2_hw.h"

namespace armadillo2_hw {

    ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh) :dxl_motors_(nh), battery_(nh), roboteq_(nh)
    {
        node_handle_ = &nh;


if (!ros::param::get(TORSO_JOINT_PARAM, torso_.joint_name))
        {
            ROS_ERROR("[armadillo2_hw/armadillo2_hw]: %s param is missing on param server. make sure that this param exist in controllers.yaml "
                              "and that your launch includes this param file. shutting down...", TORSO_JOINT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }


        servo_pub_ = nh.advertise<std_msgs::UInt16>("/ric/torso/command", 1);
        servo_sub_ = nh.subscribe("ric/torso/feedback", 1, &ArmadilloHW::torsoCallback, this);
        emergency_sub_ = nh.subscribe("ric/emergency", 1, &ArmadilloHW::emergencyCallback, this);
        /* register handles */
        dxl_motors_.registerHandles(joint_state_interface_,
                                    position_interface_,
                                    posvel_interface_);



        /* joint state registration */
        joint_state_handles_.push_back(hardware_interface::JointStateHandle(torso_.joint_name,
                                                                            &torso_.pos,
                                                                            &torso_.vel,
                                                                            &torso_.effort));
        joint_state_interface_.registerHandle(joint_state_handles_.back());
        /* joint command registration */
        pos_handles_.push_back(hardware_interface::JointHandle(joint_state_interface_.getHandle(torso_.joint_name),
                                                               &torso_.command_effort));
        effort_interface_.registerHandle(pos_handles_.back());


        roboteq_.registerHandles(joint_state_interface_,velocity_interface_);

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&posvel_interface_);
        registerInterface(&position_interface_);
        registerInterface(&velocity_interface_);
        registerInterface(&effort_interface_);

        prev_time_ = servo_pub_prev_time_ = servo_sub_prev_time_ = ros::Time::now();


        ROS_INFO("[armadillo2_hw]: armadillo hardware interface loaded successfully");
        espeak_pub_ = node_handle_->advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        speakMsg("i am ready", 1);
    }

    void ArmadilloHW::emergencyCallback(const std_msgs::Bool::ConstPtr &msg) {

        if (msg->data) {
            speakMsg("emergency, shutting down", 1);
            ROS_ERROR("[armadillo2_hw/ricboard_pub]: EMERGENCY PIN DISCONNECTED, shutting down...");
            ros::shutdown();
            exit(EXIT_FAILURE);
        }
    }

    void ArmadilloHW::torsoCallback(const std_msgs::Float32::ConstPtr &msg) {
        ros::Time now = ros::Time::now();
        if(msg->data != -1.0)
        {
            torso_.pos = msg->data;
            torso_.vel = (torso_.pos - torso_.prev_pos) / (now - servo_sub_prev_time_).toSec();
            torso_.effort = torso_.command_effort;
            torso_.prev_pos = torso_.pos;
        }
        servo_sub_prev_time_ = now;
    }

    void ArmadilloHW::read() {
        ros::Duration period = ros::Time::now() - prev_time_;
        dxl_motors_.read();
        roboteq_.read(period);

        // ric_.read(period);
    }

    void ArmadilloHW::write() {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - prev_time_;
        dxl_motors_.write();
        roboteq_.write(period);

        
        if ((now - servo_pub_prev_time_).toSec() > 0.05) //20hz
        {
            std_msgs::UInt16 msg;
            msg.data = torso_.command_effort + 1500;
            servo_pub_.publish(msg);
            servo_pub_prev_time_ = now;
        }
        prev_time_ = now;
    }


}
