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

namespace armadillo2_hw
{

    ArmadilloHW::ArmadilloHW(ros::NodeHandle &nh) :
            dxl_motors_(nh), battery_(nh), ric_(nh), roboteq_(nh)
    {
        node_handle_ = &nh;

        /* register handles */
        dxl_motors_.registerHandles(joint_state_interface_,
                                    position_interface_,
                                    posvel_interface_);
        ric_.registerHandles(joint_state_interface_,
                             effort_interface_);
        roboteq_.registerHandles(joint_state_interface_,
                                 velocity_interface_);

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&posvel_interface_);
        registerInterface(&position_interface_);
        registerInterface(&velocity_interface_);
        registerInterface(&effort_interface_);

        prev_time_ = ros::Time::now();

        ric_.startLoop();

        ROS_INFO("[armadillo2_hw]: armadillo hardware interface loaded successfully");
        espeak_pub_ = node_handle_->advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        speakMsg("i am ready", 1);
    }

    void ArmadilloHW::read()
    {
        ros::Duration period = ros::Time::now() - prev_time_;
        dxl_motors_.read();
        roboteq_.read(period);
        ric_.read(period);
    }

    void ArmadilloHW::write()
    {
        ros::Duration period = ros::Time::now() - prev_time_;
        dxl_motors_.write();
        roboteq_.write(period);
        ric_.write(period);
        prev_time_ = ros::Time::now();
    }
}
