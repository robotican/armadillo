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


#ifndef ARMADILLO2_TELEOP_ARMADILLO_JOY_H
#define ARMADILLO2_TELEOP_ARMADILLO_JOY_H

#include <ros/forwards.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "joy_profile.h"

struct joints_state_indx
{
    static const uint8_t TORSO = 11;
    static const uint8_t PAN = 0;
    static const uint8_t TILT = 1;
};

class Armadillo2Teleop
{
private:
    //ros::NodeHandle *nh_;
    ros::Publisher torso_real_pub_,
                   torso_sim_pub_,
                   twist_pub_,
                   head_pub_;
    ros::Subscriber joy_sub_,
                    joints_states_sub_,
                    gripper_sub_;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface *arm_grp_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *gripper_client_;

    joints_state_indx states_;
    joy_profile joy_;
    bool tele_arm_ = false;

    void drive();
    void moveTorso();
    void moveArm();
    void moveGripper();
    void moveHead();
    void resetArm();
    void update(const sensor_msgs::Joy::ConstPtr& joy);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    bool loadProfile(const std::string &profile_name);
    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr& msg);
    void gripperGapCB(const std_msgs::Float32::ConstPtr& msg);

public:
    Armadillo2Teleop();
    ~Armadillo2Teleop()
    {
        delete gripper_client_;
        delete arm_grp_;
    }
};

#endif //ARMADILLO2_TELEOP_ARMADILLO_JOY_H
