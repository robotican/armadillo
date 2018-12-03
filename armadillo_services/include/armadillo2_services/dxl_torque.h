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

#ifndef ARMADILLO2_SERVICES_DXL_TORQUE_H
#define ARMADILLO2_SERVICES_DXL_TORQUE_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <armadillo2_services/lift_arm.h>
#include <armadillo2_services/pan_tilt_mover.h>
#include <armadillo2_services/joints_state_reader.h>

struct head_state
{
    double pan_pos_rad = 0;
    double tilt_pos_rad = 0;
    bool got_state = false;
};

class DxlTorque
{
private:
    ros::NodeHandle* nh_;
    ros::Publisher dxl_traj_pub_;
    ros::ServiceServer set_torque_srv_;
    ros::ServiceClient set_torque_client_;
    const JointStateReader* joint_states_;


    const PanTiltMover *head_mover_;

    bool setTorqueCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void commandCurrentDxlPosition();

public:
    DxlTorque(ros::NodeHandle& nh,
              const PanTiltMover &head_mover,
              const JointStateReader &joints_state);
    bool setTorque(bool onoff);

};


#endif //ARMADILLO2_SERVICES_DXL_TORQUE_H
