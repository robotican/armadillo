#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>

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

enum ArmPose
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

ros::Publisher arm_pub;
ros::Subscriber joints_state_sub;
ros::ServiceServer lift_arm_srv;
arm_state arm;

/* check that arm is in valid start pos */
/* return: arm pose                     */
ArmPose getArmPose()
{
    //ROS_INFO("real %f: | lower: %f | upper: %f", arm.rotation1_pos_rad, (ROTATION1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE), (ROTATION1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE));
    if (arm.rotation1_pos_rad > ROTATION1_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
        arm.rotation1_pos_rad < ROTATION1_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
    {
        /*if (arm.rotation2_pos_rad > ROTATION2_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
            arm.rotation2_pos_rad < ROTATION2_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
        {
            if (arm.shoulder1_pos_rad > SHOULDER1_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                arm.shoulder1_pos_rad < SHOULDER1_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
            {
                if (arm.shoulder2_pos_rad > SHOULDER2_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                    arm.shoulder2_pos_rad < SHOULDER2_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                {
                    if (arm.shoulder3_pos_rad > SHOULDER3_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                        arm.shoulder3_pos_rad < SHOULDER3_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                    {
                        if (arm.wrist_pos_rad > WRIST_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                            arm.wrist_pos_rad < WRIST_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)*/
                            return ::GRIPPER_TO_THE_RIGHT;
                    /*}
                }
            }
        }*/
    }
    else if (arm.rotation1_pos_rad > ROTATION1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
             arm.rotation1_pos_rad < ROTATION1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
    {
        /*if (arm.rotation2_pos_rad > ROTATION2_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
            arm.rotation2_pos_rad < ROTATION2_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
        {
            if (arm.shoulder1_pos_rad > SHOULDER1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                arm.shoulder1_pos_rad < SHOULDER1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
            {
                if (arm.shoulder2_pos_rad > SHOULDER2_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                    arm.shoulder2_pos_rad < SHOULDER2_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                {
                    if (arm.shoulder3_pos_rad > SHOULDER3_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                        arm.shoulder3_pos_rad < SHOULDER3_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                    {
                        if (arm.wrist_pos_rad > WRIST_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                            arm.wrist_pos_rad < WRIST_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)*/
                            return ::GRIPPER_TO_THE_LEFT;
                    /*}
                }
            }
        }*/
    }
    return ::INVALID;
}

bool liftArmCB(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res)
{


    trajectory_msgs::JointTrajectory traj_msg;

    traj_msg.joint_names.push_back(ROTATION1_JOINT_NAME);
    traj_msg.joint_names.push_back(ROTATION2_JOINT_NAME);
    traj_msg.joint_names.push_back(SHOULDER1_JOINT_NAME);
    traj_msg.joint_names.push_back(SHOULDER2_JOINT_NAME);
    traj_msg.joint_names.push_back(SHOULDER3_JOINT_NAME);
    traj_msg.joint_names.push_back(WRIST_JOINT_NAME);

    trajectory_msgs::JointTrajectoryPoint point;

    switch (getArmPose())
    {
        case ::INVALID :
        {
            res.success = false;
            res.message = "arm is in invalid start position. move arm to valid start position, and try again";
            return true;
        }
        case ::GRIPPER_TO_THE_LEFT :
        {
            point.positions.push_back(ROTATION1_RAD_GOAL_LEFT_POS);
            point.positions.push_back(ROTATION2_RAD_GOAL_LEFT_POS);
            point.positions.push_back(SHOULDER1_RAD_GOAL_LEFT_POS);
            point.positions.push_back(SHOULDER2_RAD_GOAL_LEFT_POS);
            point.positions.push_back(SHOULDER3_RAD_GOAL_LEFT_POS);
            point.positions.push_back(WRIST_RAD_GOAL_LEFT_POS);
            res.message = "gripper to the left";
            break;
        }
        case ::GRIPPER_TO_THE_RIGHT :
        {
            point.positions.push_back(ROTATION1_RAD_GOAL_RIGHT_POS);
            point.positions.push_back(ROTATION2_RAD_GOAL_RIGHT_POS);
            point.positions.push_back(SHOULDER1_RAD_GOAL_RIGHT_POS);
            point.positions.push_back(SHOULDER2_RAD_GOAL_RIGHT_POS);
            point.positions.push_back(SHOULDER3_RAD_GOAL_RIGHT_POS);
            point.positions.push_back(WRIST_RAD_GOAL_RIGHT_POS);
            res.message = "gripper to the right";
            break;
        }
    }

    point.velocities.push_back(ROTATION1_RAD_GOAL_VEL);
    point.velocities.push_back(ROTATION2_RAD_GOAL_VEL);
    point.velocities.push_back(SHOULDER1_RAD_GOAL_VEL);
    point.velocities.push_back(SHOULDER2_RAD_GOAL_VEL);
    point.velocities.push_back(SHOULDER3_RAD_GOAL_VEL);
    point.velocities.push_back(WRIST_RAD_GOAL_VEL);

    point.time_from_start = ros::Duration(1);

    traj_msg.points.push_back(point);

    traj_msg.header.stamp = ros::Time::now();

    arm_pub.publish(traj_msg);
    res.success = true;

    return true;
}

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    /* save joints updated state */
    arm.got_state = true;
    arm.rotation1_pos_rad = msg->position[ROTATION1_JOINT_INDX];
    arm.rotation2_pos_rad = msg->position[ROTATION2_JOINT_INDX];
    arm.shoulder1_pos_rad = msg->position[SHOULDER1_JOINT_INDX];
    arm.shoulder2_pos_rad = msg->position[SHOULDER2_JOINT_INDX];
    arm.shoulder3_pos_rad = msg->position[SHOULDER3_JOINT_INDX];
    arm.wrist_pos_rad = msg->position[WRIST_JOINT_INDX];
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "lift_arm_node");
    ros::NodeHandle nh;

    arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("arm_trajectory_controller/command", 5);
    joints_state_sub = nh.subscribe("joint_states", 5, chatterCallback);
    lift_arm_srv = nh.advertiseService("lift_arm", liftArmCB);

    ros::spin();
    return 0;
}

