#ifndef ARMADILLO2_SERVICES_LIFT_ARM_H
#define ARMADILLO2_SERVICES_LIFT_ARM_H


#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/GripperCommandActionGoal.h>

#define ROTATION1_JOINT_INDX 6
#define ROTATION2_JOINT_INDX 7
#define SHOULDER1_JOINT_INDX 8
#define SHOULDER2_JOINT_INDX 9
#define SHOULDER3_JOINT_INDX 10
#define WRIST_JOINT_INDX 12

/* valid starting positions */
#define VALID_START_RAD_GOAL_TOLERANCE 0.174533 //10 deg

#define ROTATION1_VALID_START_RAD_LEFT_POS 1.567
#define ROTATION2_VALID_START_RAD_LEFT_POS 0.0
#define SHOULDER1_VALID_START_RAD_LEFT_POS -1.883
#define SHOULDER2_VALID_START_RAD_LEFT_POS 2.366
#define SHOULDER3_VALID_START_RAD_LEFT_POS 0.413287
#define WRIST_VALID_START_RAD_LEFT_POS 0.0

#define ROTATION1_VALID_START_RAD_RIGHT_POS -1.567
#define ROTATION2_VALID_START_RAD_RIGHT_POS 0.0
#define SHOULDER1_VALID_START_RAD_RIGHT_POS -1.883
#define SHOULDER2_VALID_START_RAD_RIGHT_POS 2.366
#define SHOULDER3_VALID_START_RAD_RIGHT_POS 0.413287
#define WRIST_VALID_START_RAD_RIGHT_POS 0.0

/* goal position when arm is folded with gripper */
/* on the right side of the robot                */
#define ROTATION1_RAD_GOAL_LEFT_POS 1.567
#define ROTATION2_RAD_GOAL_LEFT_POS 0.0
#define SHOULDER1_RAD_GOAL_LEFT_POS -1.5264
#define SHOULDER2_RAD_GOAL_LEFT_POS 2.1339
#define SHOULDER3_RAD_GOAL_LEFT_POS 0.0257
#define WRIST_RAD_GOAL_LEFT_POS 0.0

/* goal position when arm is folded with gripper */
/* on the left side of the robot                 */
#define ROTATION1_RAD_GOAL_RIGHT_POS -1.567
#define ROTATION2_RAD_GOAL_RIGHT_POS 0.0
#define SHOULDER1_RAD_GOAL_RIGHT_POS -1.5264
#define SHOULDER2_RAD_GOAL_RIGHT_POS 2.1339
#define SHOULDER3_RAD_GOAL_RIGHT_POS 0.0257
#define WRIST_RAD_GOAL_RIGHT_POS 0.0

#define ROTATION1_RAD_GOAL_VEL 0.1
#define ROTATION2_RAD_GOAL_VEL 0.1
#define SHOULDER1_RAD_GOAL_VEL 0.1
#define SHOULDER2_RAD_GOAL_VEL 0.1
#define SHOULDER3_RAD_GOAL_VEL 0.1
#define WRIST_RAD_GOAL_VEL 0.1

#define ROTATION1_JOINT_NAME "rotation1_joint"
#define ROTATION2_JOINT_NAME "rotation2_joint"
#define SHOULDER1_JOINT_NAME "shoulder1_joint"
#define SHOULDER2_JOINT_NAME "shoulder2_joint"
#define SHOULDER3_JOINT_NAME "shoulder3_joint"
#define WRIST_JOINT_NAME "wrist_joint"

enum class ArmPose
{
    INVALID,
    GRIPPER_TO_THE_RIGHT,
    GRIPPER_TO_THE_LEFT
};

struct arm_state
{
    double rotation1_pos_rad = 0;
    double rotation2_pos_rad = 0;
    double shoulder1_pos_rad = 0;
    double shoulder2_pos_rad = 0;
    double shoulder3_pos_rad = 0;
    double wrist_pos_rad = 0;
    bool got_state = false;
};

class LiftArm
{
private:
    ros::NodeHandle *nh_;
    ros::Publisher arm_pub_,
                   gripper_pub_;
    ros::Subscriber joints_state_sub_;
    ros::ServiceServer lift_arm_srv_;
    ros::ServiceServer open_gripper_srv_;
    arm_state arm_;

    ArmPose getArmPose();
    bool liftArmCB(std_srvs::Trigger::Request  &req,
                   std_srvs::Trigger::Response &res);
    void jointsUpdateCB(const sensor_msgs::JointState::ConstPtr& msg);
    bool openGripperCB(std_srvs::Trigger::Request  &req,
                       std_srvs::Trigger::Response &res);

public:
    LiftArm(ros::NodeHandle &nh);
};


#endif //ARMADILLO2_SERVICES_LIFT_ARM_H
