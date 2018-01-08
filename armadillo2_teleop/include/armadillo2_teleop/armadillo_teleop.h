//
// Created by armadillo2 on 27/12/17.
//

#ifndef ARMADILLO2_TELEOP_ARMADILLO_JOY_H
#define ARMADILLO2_TELEOP_ARMADILLO_JOY_H

#include <ros/forwards.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "joy_profile.h"

struct joints_state
{
    static const uint8_t INDX_TORSO = 11;
    static const uint8_t INDX_GRIPPER;
    static const uint8_t INDX_PAN;
    static const uint8_t INDX_TILT;

    double torso = 0;
    double gripper = 0;
    double pan = 0;
    double tilt = 0;
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
                    joints_states_sub_;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface arm_grp_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *gripper_client_;

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

public:
    joy_profile joy;
    Armadillo2Teleop();
    ~Armadillo2Teleop() { delete gripper_client_; }
};

#endif //ARMADILLO2_TELEOP_ARMADILLO_JOY_H
