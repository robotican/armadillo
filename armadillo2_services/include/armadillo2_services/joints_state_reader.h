//
// Created by armadillo2 on 15/01/18.
//

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
    ros::Subscriber joints_state_sub_;
    ros::NodeHandle *nh_;
    armadillo2_state armadillo_state_;
    bool got_state_ = false;

    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr& msg);

public:
    JointStateReader(ros::NodeHandle nh);
    armadillo2_state getJointsState() const { return armadillo_state_; }
    bool gotState() const { return joints_state_sub_.getNumPublishers() > 0; }
};


#endif //ARMADILLO2_SERVICES_JOINTS_STATE_READER_H
