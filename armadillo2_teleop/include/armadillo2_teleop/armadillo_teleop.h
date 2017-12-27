//
// Created by armadillo2 on 27/12/17.
//

#ifndef ARMADILLO2_TELEOP_ARMADILLO_JOY_H
#define ARMADILLO2_TELEOP_ARMADILLO_JOY_H

#include <ros/forwards.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

struct twist_joy
{
    float axis_linear = 0;
    float axis_angular = 0;

    float scale_angular = 1;
    float scale_linear = 1;

    int joy_axes_linear = 3;
    int joy_axes_angular = 2;
};

struct torso_joy
{
    float axis_updown = 0;
    float inc_updown = 0;
};

struct arm_joy
{
    float axis_rotation1 = 0;
    float axis_rotation2 = 0;
    float axis_shoulder1 = 0;
    float axis_shoulder2 = 0;
    float axis_shoulder3 = 0;
    float axis_wrist = 0;

    float inc_rotation1 = 0;
    float inc_rotation2 = 0;
    float inc_shoulder1 = 0;
    float inc_shoulder2 = 0;
    float inc_shoulder3 = 0;
    float inc_wrist = 0;
};

struct gripper_joy
{
    float axis_right_finger = 0;
    float axis_left_finger = 0;

    float inc_right_finger = 0;
    float inc_left_finger = 0;
};

struct pan_tilt_joy
{
    float axis_pan = 0;
    float axis_tilt = 0;

    float inc_pan = 0;
    float inc_tilt = 0;
};

class Armadillo2Teleop
{
private:
    ros::NodeHandle *nh_;
    ros::Publisher twist_pub_;


public:
    Armadillo2Teleop(ros::NodeHandle &nh);
    void drive(const twist_joy &twist);
    void moveTorso();
    void moveArm();
    void moveHead();
};

#endif //ARMADILLO2_TELEOP_ARMADILLO_JOY_H
