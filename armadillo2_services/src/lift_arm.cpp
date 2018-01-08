

#include <armadillo2_services/lift_arm.h>

LiftArm::LiftArm(ros::NodeHandle &nh)
{
    nh_ = &nh;
    arm_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("arm_trajectory_controller/command", 5);
    gripper_pub_ = nh.advertise<control_msgs::GripperCommandActionGoal>("gripper_controller/gripper_cmd/goal", 5);
    joints_state_sub_ = nh.subscribe("joint_states", 5, &LiftArm::jointsUpdateCB, this);
    lift_arm_srv_ = nh.advertiseService("lift_arm", &LiftArm::liftArmCB, this);
    open_gripper_srv_ = nh.advertiseService("open_gripper", &LiftArm::openGripperCB, this);
}

void LiftArm::jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    /* save joints updated state */
    arm_.got_state = true;
    arm_.rotation1_pos_rad = msg->position[ROTATION1_JOINT_INDX];
    arm_.rotation2_pos_rad = msg->position[ROTATION2_JOINT_INDX];
    arm_.shoulder1_pos_rad = msg->position[SHOULDER1_JOINT_INDX];
    arm_.shoulder2_pos_rad = msg->position[SHOULDER2_JOINT_INDX];
    arm_.shoulder3_pos_rad = msg->position[SHOULDER3_JOINT_INDX];
    arm_.wrist_pos_rad = msg->position[WRIST_JOINT_INDX];
}

bool LiftArm::liftArmCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
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
        case ArmPose::INVALID :
        {
            res.success = false;
            res.message = "arm is in invalid start position. move arm to valid start position, and try again";
            return true;
        }
        case ArmPose::GRIPPER_TO_THE_LEFT :
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
        case ArmPose::GRIPPER_TO_THE_RIGHT :
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
    arm_pub_.publish(traj_msg);

    res.success = true;
    return true;
}

bool LiftArm::openGripperCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    control_msgs::GripperCommandActionGoal gripper_msg;
    gripper_msg.header.stamp = ros::Time::now();
    gripper_msg.goal.command.position = 0.5;
    gripper_msg.goal.command.max_effort = 0.4;
    gripper_pub_.publish(gripper_msg);
    res.message = "gripper open request was sent";
    res.success = true;
    return true;
}

ArmPose LiftArm::getArmPose()
{
    //ROS_INFO("real %f: | lower: %f | upper: %f", arm.rotation1_pos_rad, (ROTATION1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE), (ROTATION1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE));
    if (arm_.rotation1_pos_rad > ROTATION1_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
            arm_.rotation1_pos_rad < ROTATION1_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
    {
        /*if (arm_.rotation2_pos_rad > ROTATION2_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
            arm_.rotation2_pos_rad < ROTATION2_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
        {
            if (arm_.shoulder1_pos_rad > SHOULDER1_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                arm_.shoulder1_pos_rad < SHOULDER1_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
            {
                if (arm_.shoulder2_pos_rad > SHOULDER2_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                    arm_.shoulder2_pos_rad < SHOULDER2_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                {
                    if (arm_.shoulder3_pos_rad > SHOULDER3_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                        arm_.shoulder3_pos_rad < SHOULDER3_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                    {
                        if (arm_.wrist_pos_rad > WRIST_VALID_START_RAD_RIGHT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                            arm_.wrist_pos_rad < WRIST_VALID_START_RAD_RIGHT_POS + VALID_START_RAD_GOAL_TOLERANCE)*/
        return ArmPose::GRIPPER_TO_THE_RIGHT;
        /* }
     }
    }
    }*/
        }
        else if (arm_.rotation1_pos_rad > ROTATION1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
            arm_.rotation1_pos_rad < ROTATION1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
        {
            /*    if (arm_.rotation2_pos_rad > ROTATION2_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                    arm_.rotation2_pos_rad < ROTATION2_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                {
                    if (arm_.shoulder1_pos_rad > SHOULDER1_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                        arm_.shoulder1_pos_rad < SHOULDER1_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                    {
                        if (arm_.shoulder2_pos_rad > SHOULDER2_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                            arm_.shoulder2_pos_rad < SHOULDER2_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                        {
                            if (arm_.shoulder3_pos_rad > SHOULDER3_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                                arm_.shoulder3_pos_rad < SHOULDER3_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)
                            {
                                if (arm_.wrist_pos_rad > WRIST_VALID_START_RAD_LEFT_POS - VALID_START_RAD_GOAL_TOLERANCE &&
                                    arm_.wrist_pos_rad < WRIST_VALID_START_RAD_LEFT_POS + VALID_START_RAD_GOAL_TOLERANCE)*/
            return ArmPose::GRIPPER_TO_THE_LEFT;
            /*}
        }
    }
    }*/
    }
    return ArmPose::INVALID;
}