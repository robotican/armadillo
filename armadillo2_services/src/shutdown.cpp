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

#include <armadillo2_services/shutdown.h>

Shutdown::Shutdown(ros::NodeHandle &nh,
                   const PanTiltMover &head_mover,
                   DxlTorque &dxl_torque)
{
    nh_ = &nh;
    dxl_torque_ = &dxl_torque;
    head_mover_ = &head_mover;

    shutdown_srv_ = nh_->advertiseService("services/shutdown", &Shutdown::shutdownCB, this);
    espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
}

bool Shutdown::shutdownCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std_msgs::String speak_msg;
    speak_msg.data = "shutting down";
    espeak_pub_.publish(speak_msg);
    /* prepare head for shutdown - lower by 40 deg */
    head_mover_->publishTrajectoryMsg(0, 0.698131);
    ros::Duration(3).sleep(); //give head time to go down
    dxl_torque_->setTorque(false);
    return true;
}