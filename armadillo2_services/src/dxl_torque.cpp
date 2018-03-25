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

#include <armadillo2_services/dxl_torque.h>


DxlTorque::DxlTorque(ros::NodeHandle& nh,
                     const PanTiltMover &head_mover,
                     const JointStateReader &joints_state)
{
    nh_ = &nh;
    joint_states_ = &joints_state;
    head_mover_ = &head_mover;
    set_torque_srv_ = nh_->advertiseService("services/dxl_torque", &DxlTorque::setTorqueCB, this);
    set_torque_client_ = nh_->serviceClient<std_srvs::SetBool>("hardware/dxl_torque");
    dxl_traj_pub_ = nh_->advertise<trajectory_msgs::JointTrajectory>("arm_trajectory_controller/command", 5);
}

bool DxlTorque::setTorque(bool onoff)
{
    std_srvs::SetBool torque_msg;
    torque_msg.request.data = onoff;
    set_torque_client_.call(torque_msg);
    return torque_msg.response.success;
}

bool DxlTorque::setTorqueCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (req.data) //if requested torque on
    {
        if (!joint_states_->gotState())
        {
            res.message = "didn't get arm or head joint state message";
            res.success = false;
            return true;
        }
        commandCurrentDxlPosition();
        ros::Duration(1).sleep();
    }
    if (!set_torque_client_.exists())
    {
        res.message = "hardware/dxl_torque service is not available. are you running in gazebo?";
        res.success = false;
        return true;
    }
    res.success = setTorque(req.data);
    return true;
}

/* command all dxl motors to stay in place, so when */
/* torque comes back on, controller won't send it   */
/* to position before torque off                    */
void DxlTorque::commandCurrentDxlPosition()
{
    armadillo2_state joints_state = joint_states_->getJointsState();

    trajectory_msgs::JointTrajectory traj_msg;

    traj_msg.joint_names.push_back(JOINT_NAME_ROTATION1);
    traj_msg.joint_names.push_back(JOINT_NAME_ROTATION2);
    traj_msg.joint_names.push_back(JOINT_NAME_SHOULDER1);
    traj_msg.joint_names.push_back(JOINT_NAME_SHOULDER2);
    traj_msg.joint_names.push_back(JOINT_NAME_SHOULDER3);
    traj_msg.joint_names.push_back(JOINT_NAME_WRIST);

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(joints_state.rotation1);
    point.positions.push_back(joints_state.rotation2);
    point.positions.push_back(joints_state.shoulder1);
    point.positions.push_back(joints_state.shoulder2);
    point.positions.push_back(joints_state.shoulder3);
    point.positions.push_back(joints_state.wrist);

    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);

    point.time_from_start = ros::Duration(1);
    traj_msg.points.push_back(point);
    traj_msg.header.stamp = ros::Time::now();
    dxl_traj_pub_.publish(traj_msg);

    head_mover_->publishTrajectoryMsg(joints_state.pan, joints_state.tilt);
}
