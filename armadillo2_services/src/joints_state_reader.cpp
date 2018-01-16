//
// Created by armadillo2 on 15/01/18.
//

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