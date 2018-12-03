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


#ifndef ARMADILLO2_SERVICES_JOINTS_STATE_READER_H
#define ARMADILLO2_SERVICES_JOINTS_STATE_READER_H

#define INDX_JOINT_PAN 0
#define INDX_JOINT_TILT 1
#define INDX_JOINT_LEFT_FINGER 2
#define INDX_JOINT_LEFT_WHEEL 3
#define INDX_JOINT_RIGHT_FINGER 4
#define INDX_JOINT_RIGHT_WHEEL 5
#define INDX_JOINT_ROTATION1 6
#define INDX_JOINT_ROTATION2 7
#define INDX_JOINT_SHOULDER1 8
#define INDX_JOINT_SHOULDER2 9
#define INDX_JOINT_SHOULDER3 10
#define INDX_JOINT_TORSO 11
#define INDX_JOIN_WRIST 12

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

struct armadillo2_state
{
    double rotation1 = 0; //rad
    double rotation2 = 0; //rad
    double shoulder1 = 0; //rad
    double shoulder2 = 0; //rad
    double shoulder3 = 0; //rad
    double wrist = 0; //rad
    double finger_left = 0; //rad
    double finger_right = 0; //rad
    double pan = 0; //rad
    double tilt = 0; //rad
    double torso = 0; //m
    double wheel_left = 0; //rad
    double wheel_right = 0; //rad

};

class JointStateReader
{
private:
    ros::NodeHandle *nh_;
    ros::Subscriber joints_state_sub_;
    armadillo2_state armadillo_state_;
    bool got_state_ = false;

    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr& msg);

public:
    JointStateReader(ros::NodeHandle nh);
    armadillo2_state getJointsState() const { return armadillo_state_; }
    bool gotState() const { return got_state_; }
};


#endif //ARMADILLO2_SERVICES_JOINTS_STATE_READER_H
