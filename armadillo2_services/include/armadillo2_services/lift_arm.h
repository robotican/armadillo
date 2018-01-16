#ifndef ARMADILLO2_SERVICES_LIFT_ARM_H
#define ARMADILLO2_SERVICES_LIFT_ARM_H


#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Trigger.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <armadillo2_services/joints_state_reader.h>

/* valid starting positions (rad) */
#define START_TOLERANCE_ROTATION1 0.174533 //10 deg
#define START_TOLERANCE_SHOULDER1 0.523599 //30 deg
#define START_TOLERANCE_SHOULDER2 0.523599
#define START_TOLERANCE_ROTATION2 0.523599
#define START_TOLERANCE_SHOULDER3 0.523599
#define START_TOLERANCE_WRIST     0.523599

#define VALID_START_POSE_LEFT_ROTATION1 1.57
#define VALID_START_POSE_LEFT_SHOULDER1 -1.57
#define VALID_START_POSE_LEFT_SHOULDER2 2.3522
#define VALID_START_POSE_LEFT_ROTATION2 0.0
#define VALID_START_POSE_LEFT_SHOULDER3 0.5348
#define VALID_START_POSE_LEFT_WRIST     0.0

#define VALID_START_POSE_RIGHT_ROTATION1 -1.57
#define VALID_START_POSE_RIGHT_SHOULDER1 -1.57
#define VALID_START_POSE_RIGHT_SHOULDER2 2.3522
#define VALID_START_POSE_RIGHT_ROTATION2 0.0
#define VALID_START_POSE_RIGHT_SHOULDER3 0.5348
#define VALID_START_POSE_RIGHT_WRIST     0.0

/* goal position when arm is folded with gripper */
/* on the left side of the robot (rad)           */
#define GOAL_POSE_LEFT_ROTATION1 1.57
#define GOAL_POSE_LEFT_SHOULDER1 -1.2992
#define GOAL_POSE_LEFT_SHOULDER2 2.0785
#define GOAL_POSE_LEFT_ROTATION2 0.0
#define GOAL_POSE_LEFT_SHOULDER3 0.0
#define GOAL_POSE_LEFT_WRIST     0.0

/* goal position when arm is folded with gripper */
/* on the right side of the robot (rad)           */
#define GOAL_POSE_RIGHT_ROTATION1 -1.57
#define GOAL_POSE_RIGHT_SHOULDER1 -1.2992
#define GOAL_POSE_RIGHT_SHOULDER2 2.0785
#define GOAL_POSE_RIGHT_ROTATION2 0.0
#define GOAL_POSE_RIGHT_SHOULDER3 0.0
#define GOAL_POSE_RIGHT_WRIST     0.0

#define GOAL_VEL_ROTATION1 0.1
#define GOAL_VEL_ROTATION2 0.1
#define GOAL_VEL_SHOULDER1 0.1
#define GOAL_VEL_SHOULDER2 0.1
#define GOAL_VEL_SHOULDER3 0.1
#define GOAL_VEL_WRIST     0.1

#define JOINT_NAME_ROTATION1 "rotation1_joint"
#define JOINT_NAME_ROTATION2 "rotation2_joint"
#define JOINT_NAME_SHOULDER1 "shoulder1_joint"
#define JOINT_NAME_SHOULDER2 "shoulder2_joint"
#define JOINT_NAME_SHOULDER3 "shoulder3_joint"
#define JOINT_NAME_WRIST     "wrist_joint"

enum class ArmPose
{
    INVALID,
    GRIPPER_TO_THE_RIGHT,
    GRIPPER_TO_THE_LEFT
};

class LiftArm
{
private:
    ros::NodeHandle* nh_;
    ros::Publisher arm_pub_,
                   gripper_pub_;
    ros::ServiceServer lift_arm_srv_;
    ros::ServiceServer open_gripper_srv_;
    const JointStateReader* joint_states_;

    ArmPose getArmPose();
    bool liftArmCB(std_srvs::Trigger::Request  &req,
                   std_srvs::Trigger::Response &res);

    bool openGripperCB(std_srvs::Trigger::Request  &req,
                       std_srvs::Trigger::Response &res);

public:
    LiftArm(ros::NodeHandle &nh,
            const JointStateReader &joints_state);
};


#endif //ARMADILLO2_SERVICES_LIFT_ARM_H
