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


#include <armadillo2_services/lift_arm.h>

LiftArm::LiftArm(ros::NodeHandle &nh, const JointStateReader &joints_state)
{
    nh_ = &nh;
    joint_states_ = &joints_state;
    arm_pub_ = nh_->advertise<trajectory_msgs::JointTrajectory>("arm_trajectory_controller/command", 5);
    gripper_pub_ = nh_->advertise<control_msgs::GripperCommandActionGoal>("gripper_controller/gripper_cmd/goal", 5);
    lift_arm_srv_ = nh_->advertiseService("services/lift_arm", &LiftArm::liftArmCB, this);
    open_gripper_srv_ = nh_->advertiseService("services/open_gripper", &LiftArm::openGripperCB, this);
}

bool LiftArm::liftArmCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{


    trajectory_msgs::JointTrajectory traj_msg;

    traj_msg.joint_names.push_back(JOINT_NAME_ROTATION1);
    traj_msg.joint_names.push_back(JOINT_NAME_ROTATION2);
    traj_msg.joint_names.push_back(JOINT_NAME_SHOULDER1);
    traj_msg.joint_names.push_back(JOINT_NAME_SHOULDER2);
    traj_msg.joint_names.push_back(JOINT_NAME_SHOULDER3);
    traj_msg.joint_names.push_back(JOINT_NAME_WRIST);

    trajectory_msgs::JointTrajectoryPoint point;

    switch (getArmPose())
    {
        case ArmPose::INVALID :
        {
            res.success = false;
            res.message = "arm is in invalid start position. move arm to valid start position, and try again";
            return true;
        }
        case ArmPose::GRIPPER_TO_THE_LEFT :
        {
            point.positions.push_back(GOAL_POSE_LEFT_ROTATION1);
            point.positions.push_back(GOAL_POSE_LEFT_ROTATION2);
            point.positions.push_back(GOAL_POSE_LEFT_SHOULDER1);
            point.positions.push_back(GOAL_POSE_LEFT_SHOULDER2);
            point.positions.push_back(GOAL_POSE_LEFT_SHOULDER3);
            point.positions.push_back(GOAL_POSE_LEFT_WRIST);
            res.message = "gripper to the left";
            break;
        }
        case ArmPose::GRIPPER_TO_THE_RIGHT :
        {
            point.positions.push_back(GOAL_POSE_RIGHT_ROTATION1);
            point.positions.push_back(GOAL_POSE_RIGHT_ROTATION2);
            point.positions.push_back(GOAL_POSE_RIGHT_SHOULDER1);
            point.positions.push_back(GOAL_POSE_RIGHT_SHOULDER2);
            point.positions.push_back(GOAL_POSE_RIGHT_SHOULDER3);
            point.positions.push_back(GOAL_POSE_RIGHT_WRIST);
            res.message = "gripper to the right";
            break;
        }
    }

    point.velocities.push_back(GOAL_VEL_ROTATION1);
    point.velocities.push_back(GOAL_VEL_ROTATION2);
    point.velocities.push_back(GOAL_VEL_SHOULDER1);
    point.velocities.push_back(GOAL_VEL_SHOULDER2);
    point.velocities.push_back(GOAL_VEL_SHOULDER3);
    point.velocities.push_back(GOAL_VEL_WRIST);

    point.time_from_start = ros::Duration(1);
    traj_msg.points.push_back(point);
    traj_msg.header.stamp = ros::Time::now();
    arm_pub_.publish(traj_msg);

    res.success = true;
    return true;
}

bool LiftArm::openGripperCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    control_msgs::GripperCommandActionGoal gripper_msg;
    gripper_msg.header.stamp = ros::Time::now();
    gripper_msg.goal.command.position = 0.05;
    gripper_msg.goal.command.max_effort = 0.4;
    gripper_pub_.publish(gripper_msg);
    res.message = "gripper open request was sent";
    res.success = true;
    return true;
}

ArmPose LiftArm::getArmPose()
{
    armadillo2_state joints_state = joint_states_->getJointsState();
    //ROS_INFO("real %f: | lower: %f | upper: %f", arm.rotation1_pos_rad, (ROTATION1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE), (ROTATION1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE));
    if (joints_state.rotation1 > VALID_START_POSE_LEFT_ROTATION1 - START_TOLERANCE_ROTATION1 &&
            joints_state.rotation1 < VALID_START_POSE_LEFT_ROTATION1 + START_TOLERANCE_ROTATION1 )
    {
        if (joints_state.rotation2 > VALID_START_POSE_LEFT_ROTATION2 - START_TOLERANCE_ROTATION2  &&
                joints_state.rotation2 < VALID_START_POSE_LEFT_ROTATION2 + START_TOLERANCE_ROTATION2 )
        {
            if (joints_state.shoulder1 > VALID_START_POSE_LEFT_SHOULDER1 - START_TOLERANCE_SHOULDER1 &&
                joints_state.shoulder1 < VALID_START_POSE_LEFT_SHOULDER1 + START_TOLERANCE_SHOULDER1)
            {
                if (joints_state.shoulder2 > VALID_START_POSE_LEFT_SHOULDER2 - START_TOLERANCE_SHOULDER2 &&
                    joints_state.shoulder2 < VALID_START_POSE_LEFT_SHOULDER2 + START_TOLERANCE_SHOULDER2)
                {
                    if (joints_state.shoulder3 > VALID_START_POSE_LEFT_SHOULDER3 - START_TOLERANCE_SHOULDER3 &&
                        joints_state.shoulder3 < VALID_START_POSE_LEFT_SHOULDER3 + START_TOLERANCE_SHOULDER3)
                    {
                        if (joints_state.wrist > VALID_START_POSE_LEFT_WRIST - START_TOLERANCE_WRIST &&
                            joints_state.wrist < VALID_START_POSE_LEFT_WRIST + START_TOLERANCE_WRIST)
                            return ArmPose::GRIPPER_TO_THE_LEFT;
                    }
                }
            }
        }
    }
    else if (joints_state.rotation1 > VALID_START_POSE_RIGHT_ROTATION1 - START_TOLERANCE_ROTATION1 &&
             joints_state.rotation1 < VALID_START_POSE_RIGHT_ROTATION1 + START_TOLERANCE_ROTATION1 )
    {
        if (joints_state.rotation2 > VALID_START_POSE_RIGHT_ROTATION2 - START_TOLERANCE_ROTATION2  &&
            joints_state.rotation2 < VALID_START_POSE_RIGHT_ROTATION2 + START_TOLERANCE_ROTATION2 )
        {
            if (joints_state.shoulder1 > VALID_START_POSE_RIGHT_SHOULDER1 - START_TOLERANCE_SHOULDER1 &&
                joints_state.shoulder1 < VALID_START_POSE_RIGHT_SHOULDER1 + START_TOLERANCE_SHOULDER1)
            {
                if (joints_state.shoulder2 > VALID_START_POSE_RIGHT_SHOULDER2 - START_TOLERANCE_SHOULDER2 &&
                    joints_state.shoulder2 < VALID_START_POSE_RIGHT_SHOULDER2 + START_TOLERANCE_SHOULDER2)
                {
                    if (joints_state.shoulder3 > VALID_START_POSE_RIGHT_SHOULDER3 - START_TOLERANCE_SHOULDER3 &&
                        joints_state.shoulder3 < VALID_START_POSE_RIGHT_SHOULDER3 + START_TOLERANCE_SHOULDER3)
                    {
                        if (joints_state.wrist > VALID_START_POSE_RIGHT_WRIST - START_TOLERANCE_WRIST &&
                            joints_state.wrist < VALID_START_POSE_RIGHT_WRIST + START_TOLERANCE_WRIST)
                            return ArmPose::GRIPPER_TO_THE_RIGHT;
                    }
                }
            }
        }
    }
    return ArmPose::INVALID;
}
