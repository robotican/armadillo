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

#include <armadillo2_services/joints_state_reader.h>

JointStateReader::JointStateReader(ros::NodeHandle nh)
{
    nh_ = &nh;
    joints_state_sub_ = nh_->subscribe("joint_states", 5, &JointStateReader::jointsUpdateCB, this);
}

void JointStateReader::jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    /* save joints updated state */
    got_state_ = true;
    armadillo_state_.pan = msg->position[INDX_JOINT_PAN];
    armadillo_state_.tilt = msg->position[INDX_JOINT_TILT];
    armadillo_state_.finger_left = msg->position[INDX_JOINT_LEFT_FINGER];
    armadillo_state_.wheel_left = msg->position[INDX_JOINT_LEFT_WHEEL];
    armadillo_state_.finger_right = msg->position[INDX_JOINT_RIGHT_FINGER];
    armadillo_state_.wheel_right = msg->position[INDX_JOINT_RIGHT_WHEEL];
    armadillo_state_.rotation1 = msg->position[INDX_JOINT_ROTATION1];
    armadillo_state_.rotation2 = msg->position[INDX_JOINT_ROTATION2];
    armadillo_state_.shoulder1 = msg->position[INDX_JOINT_SHOULDER1];
    armadillo_state_.shoulder2 = msg->position[INDX_JOINT_SHOULDER2];
    armadillo_state_.shoulder3 = msg->position[INDX_JOINT_SHOULDER3];
    armadillo_state_.torso = msg->position[INDX_JOINT_TORSO];
    armadillo_state_.wrist = msg->position[INDX_JOIN_WRIST];

}