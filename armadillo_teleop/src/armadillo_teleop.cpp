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

#include <armadillo_teleop/armadillo_teleop.h>
#include <armadillo_teleop/joy_profile.h>


ArmadilloTeleop::ArmadilloTeleop()
{
    std::string driving_topic,
            torso_elevator_sim_topic,
            torso_elevator_real_topic,
            head_topic,
            gripper_topic;

    ros::param::get("~tele_arm", tele_arm_);

    if (!ros::param::get("~topics/driving", driving_topic) ||
        !ros::param::get("~topics/torso/real", torso_elevator_real_topic) ||
        !ros::param::get("~topics/torso/sim", torso_elevator_sim_topic) ||
        !ros::param::get("~topics/head", head_topic) ||
        !ros::param::get("~topics/gripper", gripper_topic))
    {
        ROS_ERROR("[armadillo_teleop]: topics params are missing. did you load topics.yaml?");
        exit(EXIT_FAILURE);
    }
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(driving_topic, 5);
    torso_real_pub_ = nh_.advertise<std_msgs::Float64>(torso_elevator_real_topic, 5);
    torso_sim_pub_ = nh_.advertise<std_msgs::Float64>(torso_elevator_sim_topic, 5);
    head_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(head_topic, 5);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmadilloTeleop::joyCallback, this);
    joints_states_sub_ = nh_.subscribe("joint_states", 5, &ArmadilloTeleop::jointsUpdateCB, this);
    gripper_sub_ = nh_.subscribe("/gripper_controller/current_gap", 5, &ArmadilloTeleop::gripperGapCB, this);

    /// /gripper_controller/current_gap

    /* limits */
    ros::param::get("~limits/torso/lower", joy_.torso.limit_lower);
    ros::param::get("~limits/torso/upper", joy_.torso.limit_upper);

    ros::param::get("~limits/head/pan_lower", joy_.head.limit_lower_pan);
    ros::param::get("~limits/head/pan_upper", joy_.head.limit_upper_pan);
    ros::param::get("~limits/head/tilt_lower", joy_.head.limit_lower_tilt);
    ros::param::get("~limits/head/tilt_upper", joy_.head.limit_upper_tilt);

    ros::param::get("~limits/gripper/limit_lower", joy_.gripper.limit_lower);
    ros::param::get("~limits/gripper/limit_upper", joy_.gripper.limit_upper);
    ros::param::get("~limits/gripper/max_effort", joy_.gripper.goal.command.max_effort);

    if (tele_arm_)
    {
        arm_grp_ = new moveit::planning_interface::MoveGroupInterface("arm");
        /* move arm to start pos */
        ros::param::get("~start_pos", joy_.arm.start_pos);
        resetArm();

        /* gripper action client */
        gripper_client_ = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_topic, true);
        //actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client(gripper_topic, true);
        ROS_INFO("[armadillo_teleop]: waiting for gripper action server to start...");
        /* wait for the action server to start */
        gripper_client_->waitForServer(); //will wait for infinite time

        ROS_INFO("[armadillo_teleop]: gripper action server started");
    }

    /* load joystick profile */
    std::string profile_name = "xbox";
    ros::param::get("~profile", profile_name);
    loadProfile(profile_name);

    ROS_INFO("[armadillo_teleop]: ready to dance according to joy profile: %s", profile_name.c_str());
}

void ArmadilloTeleop::gripperGapCB(const std_msgs::Float32::ConstPtr &msg)
{
    if (!tele_arm_) return;
    joy_.gripper.goal.command.position = msg->data;
    joy_.gripper.init_state_recorded = true;
    gripper_sub_.shutdown();
}

void ArmadilloTeleop::jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    /* save joints updated state */
    joy_.head.axis_val_pan = msg->position[joints_state_indx::PAN];
    joy_.head.axis_val_tilt = msg->position[joints_state_indx::TILT];
    joy_.torso.axis_val_updown = msg->position[joints_state_indx::TORSO];
    joy_.torso.init_state_recorded = true;
    joy_.head.init_state_recorded = true;
    joints_states_sub_.shutdown();
}

void ArmadilloTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    update(joy);
}

void ArmadilloTeleop::drive()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = joy_.twist.axis_val_angular;
    twist_msg.linear.x = joy_.twist.axis_val_linear;
    twist_pub_.publish(twist_msg);
}

void ArmadilloTeleop::moveTorso()
{
    std_msgs::Float64 torso_pos;
    torso_pos.data = joy_.torso.axis_val_updown;
    torso_real_pub_.publish(torso_pos);
    torso_sim_pub_.publish(torso_pos);
}

void ArmadilloTeleop::moveHead()
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=joy_.head.axis_val_pan * M_PI / 180;
    q_goal[1]=joy_.head.axis_val_tilt * M_PI / 180;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    head_pub_.publish(traj);
}

bool ArmadilloTeleop::loadProfile(const std::string &profile_name)
{
    /* twist */
    ros::param::get("~" + profile_name + "/twist/joy_axis_linear", joy_.twist.joy_axis_linear);
    ros::param::get("~" + profile_name + "/twist/joy_axis_angular", joy_.twist.joy_axis_angular);
    ros::param::get("~" + profile_name + "/twist/scale_angular", joy_.twist.scale_angular);
    ros::param::get("~" + profile_name + "/twist/scale_linear", joy_.twist.scale_linear);
    /* torso */
    ros::param::get("~" + profile_name + "/torso/joy_axis_updown", joy_.torso.joy_axis_updown);
    ros::param::get("~" + profile_name + "/torso/increment", joy_.torso.increment);

    /* head */
    ros::param::get("~" + profile_name + "/head/right_btn", joy_.head.joy_btn_pan_right);
    ros::param::get("~" + profile_name + "/head/left_btn", joy_.head.joy_btn_pan_left);
    ros::param::get("~" + profile_name + "/head/up_btn", joy_.head.joy_btn_tilt_up);
    ros::param::get("~" + profile_name + "/head/down_btn", joy_.head.joy_btn_tilt_down);
    ros::param::get("~" + profile_name + "/head/inc_pan", joy_.head.inc_pan);
    ros::param::get("~" + profile_name + "/head/inc_tilt", joy_.head.inc_tilt);

    /* arm */
    ros::param::get("~" + profile_name + "/arm/rotation1_axis", joy_.arm.joy_axis_rotation1);
    ros::param::get("~" + profile_name + "/arm/shoulder1_axis", joy_.arm.joy_axis_shoulder1);
    ros::param::get("~" + profile_name + "/arm/shoulder2_axis", joy_.arm.joy_axis_shoulder2);
    ros::param::get("~" + profile_name + "/arm/rotation2_axis", joy_.arm.joy_axis_rotation2);
    ros::param::get("~" + profile_name + "/arm/shoulder3_up_btn", joy_.arm.joy_btn_shoulder3_up);
    ros::param::get("~" + profile_name + "/arm/shoulder3_down_btn", joy_.arm.joy_btn_shoulder3_down);
    ros::param::get("~" + profile_name + "/arm/wrist_cw_btn", joy_.arm.joy_btn_wrist_cw);
    ros::param::get("~" + profile_name + "/arm/wrist_ccw_btn", joy_.arm.joy_btn_wrist_ccw);
    ros::param::get("~" + profile_name + "/arm/reset_arm", joy_.arm.joy_btn_reset);
    ros::param::get("~" + profile_name + "/arm/inc", joy_.arm.increment);

    /* gripper */
    ros::param::get("~" + profile_name + "/gripper/axis", joy_.gripper.joy_axis);
    ros::param::get("~" + profile_name + "/gripper/inc", joy_.gripper.increment);

    /* utils */
    ros::param::get("~" + profile_name + "/arm_mode_btn", joy_.utils.joy_btn_arm_mode);
    ros::param::get("~" + profile_name + "/safety_btn", joy_.utils.joy_btn_safety);
}

void ArmadilloTeleop::moveGripper()
{
    if (!tele_arm_) return;
    gripper_client_->sendGoal(joy_.gripper.goal);
}

void ArmadilloTeleop::moveArm()
{
    if (!tele_arm_) return;
    arm_grp_->setJointValueTarget(joy_.arm.axes_vals);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (arm_grp_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_WARN("[armadillo_teleop]: moving arm...");
        arm_grp_->move();
        for (int i=0; i < joy_arm::DOF; i++)
            joy_.arm.axes_vals_prev[i] = joy_.arm.axes_vals[i];
    }
    else
    {
        ROS_WARN("[armadillo_teleop]: invalid moveit goal");
        /* if fail, revert to prev values */
        for (int i=0; i < joy_arm::DOF; i++)
            joy_.arm.axes_vals[i] = joy_.arm.axes_vals_prev[i];
    }
}

void ArmadilloTeleop::resetArm()
{
    if (!tele_arm_) return;
    arm_grp_->setPlannerId("RRTConnectkConfigDefault");
    arm_grp_->setPlanningTime(5.0);
    arm_grp_->setNumPlanningAttempts(15);
    arm_grp_->setPoseReferenceFrame("base_footprint");
    arm_grp_->setStartStateToCurrentState();
    arm_grp_->setNamedTarget(joy_.arm.start_pos);
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if(arm_grp_->plan(start_plan))  //Check if plan is valid
    {
        ROS_INFO("[armadillo_teleop]: moving arm to start position: %s", joy_.arm.start_pos.c_str());
        arm_grp_->execute(start_plan);
    }
    else
        ROS_WARN("[armadillo_teleop]: failed moving arm to start position: %s", joy_.arm.start_pos.c_str());

    arm_grp_->getCurrentState()->copyJointGroupPositions(
            arm_grp_->getCurrentState()->getRobotModel()->getJointModelGroup(arm_grp_->getName()),
            joy_.arm.axes_vals);
    joy_.arm.init_state_recorded = true;

    /* give arm some time */
    ros::Duration(3).sleep();
}


void ArmadilloTeleop::update(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    if (!joy_msg->buttons[joy_.utils.joy_btn_safety])
        return;

    if (!joy_msg->buttons[joy_.utils.joy_btn_arm_mode] &&
        joy_.torso.init_state_recorded &&
        joy_.head.init_state_recorded)
    {
        /* drive robot */
        joy_.twist.axis_val_angular = joy_msg->axes[joy_.twist.joy_axis_angular];
        joy_.twist.axis_val_angular *= joy_.twist.scale_angular;

        joy_.twist.axis_val_linear = joy_msg->axes[joy_.twist.joy_axis_linear];
        joy_.twist.axis_val_angular *= joy_.twist.scale_linear;
        drive();

        /* move torso */
        if (joy_msg->axes[joy_.torso.joy_axis_updown] == 1)
        {
            joy_.torso.axis_val_updown += joy_.torso.increment;
            if (joy_.torso.axis_val_updown > joy_.torso.limit_upper)
                joy_.torso.axis_val_updown = joy_.torso.limit_upper;
        }
        else if (joy_msg->axes[joy_.torso.joy_axis_updown] == -1)
        {

            joy_.torso.axis_val_updown -= joy_.torso.increment;
            if (joy_.torso.axis_val_updown < joy_.torso.limit_lower)
                joy_.torso.axis_val_updown = joy_.torso.limit_lower;
        }
        if (joy_msg->axes[joy_.torso.joy_axis_updown] != 0)
            moveTorso();

        /* move head */
        if (joy_msg->buttons[joy_.head.joy_btn_pan_left])
            joy_.head.axis_val_pan += joy_.head.inc_pan;
        if (joy_.head.axis_val_pan < joy_.head.limit_lower_pan)
            joy_.head.axis_val_pan = joy_.head.limit_lower_pan;

        if (joy_msg->buttons[joy_.head.joy_btn_pan_right])
            joy_.head.axis_val_pan -= joy_.head.inc_pan;
        if (joy_.head.axis_val_pan > joy_.head.limit_upper_pan)
            joy_.head.axis_val_pan = joy_.head.limit_upper_pan;

        if (joy_msg->buttons[joy_.head.joy_btn_tilt_down])
            joy_.head.axis_val_tilt += joy_.head.inc_tilt;
        if (joy_.head.axis_val_tilt < joy_.head.limit_lower_tilt)
            joy_.head.axis_val_tilt = joy_.head.limit_lower_tilt;

        if (joy_msg->buttons[joy_.head.joy_btn_tilt_up])
            joy_.head.axis_val_tilt -= joy_.head.inc_tilt;
        if (joy_.head.axis_val_tilt > joy_.head.limit_upper_tilt)
            joy_.head.axis_val_tilt = joy_.head.limit_upper_tilt;

        if (joy_msg->buttons[joy_.head.joy_btn_pan_left] ||
            joy_msg->buttons[joy_.head.joy_btn_pan_right] ||
            joy_msg->buttons[joy_.head.joy_btn_tilt_down] ||
            joy_msg->buttons[joy_.head.joy_btn_tilt_up])
            moveHead();
    }
        /* arm mode */
    else if (tele_arm_ &&
             joy_.arm.init_state_recorded &&
             joy_.gripper.init_state_recorded)
    {
        /* move gripper */
        if (joy_msg->axes[joy_.gripper.joy_axis] == 1)
        {
            joy_.gripper.goal.command.position += joy_.gripper.increment;
            if (joy_.gripper.goal.command.position > joy_.gripper.limit_upper)
                joy_.gripper.goal.command.position = joy_.gripper.limit_upper;
        }

        else if (joy_msg->axes[joy_.gripper.joy_axis] == -1)
        {
            joy_.gripper.goal.command.position -= joy_.gripper.increment;
            if (joy_.gripper.goal.command.position < joy_.gripper.limit_lower)
                joy_.gripper.goal.command.position = joy_.gripper.limit_lower;
        }
        if (joy_msg->axes[joy_.gripper.joy_axis] != 0)
            moveGripper();
        //fprintf(stderr, "pos: %f\n", joy_.gripper.goal.command.position);
        //fprintf(stderr, "limit_lower: %f\n", joy_.gripper.limit_lower);
        //fprintf(stderr, "limit_upper: %f\n", joy_.gripper.limit_upper);

        /* move arm */
        joy_.arm.axes_vals[joy_arm::INDX_ROTATION1] += joy_msg->axes[joy_.arm.joy_axis_rotation1] * joy_.arm.increment;
        joy_.arm.axes_vals[joy_arm::INDX_SHOULDER1] -= joy_msg->axes[joy_.arm.joy_axis_shoulder1] * joy_.arm.increment;
        joy_.arm.axes_vals[joy_arm::INDX_SHOULDER2] -= joy_msg->axes[joy_.arm.joy_axis_shoulder2] * joy_.arm.increment;

        joy_.arm.axes_vals[joy_arm::INDX_ROTATION2] += joy_msg->axes[joy_.arm.joy_axis_rotation2] * joy_.arm.increment;

        if (joy_msg->buttons[joy_.arm.joy_btn_shoulder3_up])
            joy_.arm.axes_vals[joy_arm::INDX_SHOULDER3] -= joy_.arm.increment;
        else if (joy_msg->buttons[joy_.arm.joy_btn_shoulder3_down])
            joy_.arm.axes_vals[joy_arm::INDX_SHOULDER3] += joy_.arm.increment;

        if (joy_msg->buttons[joy_.arm.joy_btn_wrist_cw])
            joy_.arm.axes_vals[joy_arm::INDX_WRIST] += joy_.arm.increment;
        else if (joy_msg->buttons[joy_.arm.joy_btn_wrist_ccw])
            joy_.arm.axes_vals[joy_arm::INDX_WRIST] -= joy_.arm.increment;

        if (joy_msg->axes[joy_.arm.joy_axis_rotation1] ||
            joy_msg->axes[joy_.arm.joy_axis_shoulder1] ||
            joy_msg->axes[joy_.arm.joy_axis_shoulder2] ||
            joy_msg->axes[joy_.arm.joy_axis_rotation2] ||
            joy_msg->buttons[joy_.arm.joy_btn_shoulder3_up] ||
            joy_msg->buttons[joy_.arm.joy_btn_shoulder3_down] ||
            joy_msg->buttons[joy_.arm.joy_btn_wrist_cw] ||
            joy_msg->buttons[joy_.arm.joy_btn_wrist_ccw])
            moveArm();

        if (joy_msg->buttons[joy_.arm.joy_btn_reset])
            resetArm();
    }
}


