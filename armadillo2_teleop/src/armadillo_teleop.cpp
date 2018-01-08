#include <armadillo2_teleop/armadillo_teleop.h>
#include <armadillo2_teleop/joy_profile.h>


Armadillo2Teleop::Armadillo2Teleop() : arm_grp_("arm")
{
    std::string driving_topic,
            torso_elevator_sim_topic,
            torso_elevator_real_topic,
            head_topic,
            gripper_topic;
    if (!ros::param::get("~topics/driving", driving_topic) ||
        !ros::param::get("~topics/torso/real", torso_elevator_real_topic) ||
        !ros::param::get("~topics/torso/sim", torso_elevator_sim_topic) ||
        !ros::param::get("~topics/head", head_topic) ||
        !ros::param::get("~topics/gripper", gripper_topic))
    {
        ROS_ERROR("[armadillo2_teleop]: topics params are missing. did you load topics.yaml?");
        exit(EXIT_FAILURE);
    }
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(driving_topic, 5);
    torso_real_pub_ = nh_.advertise<std_msgs::Float64>(torso_elevator_real_topic, 5);
    torso_sim_pub_ = nh_.advertise<std_msgs::Float64>(torso_elevator_sim_topic, 5);
    head_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(head_topic, 5);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Armadillo2Teleop::joyCallback, this);

    /* limits */
    ros::param::get("~limits/torso/lower", joy.torso.limit_lower);
    ros::param::get("~limits/torso/upper", joy.torso.limit_upper);

    ros::param::get("~limits/head/pan_lower", joy.head.limit_lower_pan);
    ros::param::get("~limits/head/pan_upper", joy.head.limit_upper_pan);
    ros::param::get("~limits/head/tilt_lower", joy.head.limit_lower_tilt);
    ros::param::get("~limits/head/tilt_upper", joy.head.limit_upper_tilt);

    /* moveit */
    ros::param::get("~start_pos", joy.arm.start_pos);

    arm_grp_.setPlannerId("RRTConnectkConfigDefault");
    arm_grp_.setPlanningTime(5.0);
    arm_grp_.setNumPlanningAttempts(15);
    arm_grp_.setPoseReferenceFrame("base_footprint");
    arm_grp_.setStartStateToCurrentState();
    arm_grp_.setNamedTarget(joy.arm.start_pos);
    resetArm();

    /* gripper action client */
    gripper_client_ = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_topic, true);
    //actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client(gripper_topic, true);
    ROS_INFO("[armadillo2_teleop]: waiting for gripper action server to start...");
    /* wait for the action server to start */
    gripper_client_->waitForServer(); //will wait for infinite time

    ROS_INFO("[armadillo2_teleop]: gripper action server started");

    /* load joystick profile */
    std::string profile_name = "xbox";
    ros::param::get("~profile", profile_name);
    loadProfile(profile_name);

    ROS_INFO("[armadillo2_teleop]: ready to dance according to joy profile: %s", profile_name.c_str());
}

void Armadillo2Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    update(joy);
}

void Armadillo2Teleop::drive()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = joy.twist.axis_val_angular;
    twist_msg.linear.x = joy.twist.axis_val_linear;
    twist_pub_.publish(twist_msg);
}

void Armadillo2Teleop::moveTorso()
{
    std_msgs::Float64 torso_pos;
    torso_pos.data = joy.torso.axis_val_updown;
    torso_real_pub_.publish(torso_pos);
    torso_sim_pub_.publish(torso_pos);
}

void Armadillo2Teleop::moveHead()
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=joy.head.axis_val_pan * M_PI / 180;
    q_goal[1]=joy.head.axis_val_tilt * M_PI / 180;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    head_pub_.publish(traj);
}

bool Armadillo2Teleop::loadProfile(const std::string &profile_name)
{
    /* twist */
    ros::param::get("~" + profile_name + "/twist/joy_axis_linear", joy.twist.joy_axis_linear);
    ros::param::get("~" + profile_name + "/twist/joy_axis_angular", joy.twist.joy_axis_angular);
    ros::param::get("~" + profile_name + "/twist/scale_angular", joy.twist.scale_angular);
    ros::param::get("~" + profile_name + "/twist/scale_linear", joy.twist.scale_linear);
    /* torso */
    ros::param::get("~" + profile_name + "/torso/joy_axis_updown", joy.torso.joy_axis_updown);
    ros::param::get("~" + profile_name + "/torso/increment", joy.torso.increment);

    /* head */
    ros::param::get("~" + profile_name + "/head/right_btn", joy.head.joy_btn_pan_right);
    ros::param::get("~" + profile_name + "/head/left_btn", joy.head.joy_btn_pan_left);
    ros::param::get("~" + profile_name + "/head/up_btn", joy.head.joy_btn_tilt_up);
    ros::param::get("~" + profile_name + "/head/down_btn", joy.head.joy_btn_tilt_down);
    ros::param::get("~" + profile_name + "/head/inc_pan", joy.head.inc_pan);
    ros::param::get("~" + profile_name + "/head/inc_tilt", joy.head.inc_tilt);

    /* arm */
    ros::param::get("~" + profile_name + "/arm/rotation1_axis", joy.arm.joy_axis_rotation1);
    ros::param::get("~" + profile_name + "/arm/shoulder1_axis", joy.arm.joy_axis_shoulder1);
    ros::param::get("~" + profile_name + "/arm/shoulder2_axis", joy.arm.joy_axis_shoulder2);
    ros::param::get("~" + profile_name + "/arm/rotation2_axis", joy.arm.joy_axis_rotation2);
    ros::param::get("~" + profile_name + "/arm/shoulder3_up_btn", joy.arm.joy_btn_shoulder3_up);
    ros::param::get("~" + profile_name + "/arm/shoulder3_down_btn", joy.arm.joy_btn_shoulder3_down);
    ros::param::get("~" + profile_name + "/arm/wrist_cw_btn", joy.arm.joy_btn_wrist_cw);
    ros::param::get("~" + profile_name + "/arm/wrist_ccw_btn", joy.arm.joy_btn_wrist_ccw);
    ros::param::get("~" + profile_name + "/arm/reset_arm", joy.arm.joy_btn_reset);
    ros::param::get("~" + profile_name + "/arm/inc", joy.arm.increment);

    /* gripper */
    ros::param::get("~" + profile_name + "/gripper/axis", joy.gripper.joy_axis);
    ros::param::get("~" + profile_name + "/gripper/inc", joy.gripper.increment);
    ros::param::get("~" + profile_name + "/gripper/limit_lower", joy.gripper.limit_lower);
    ros::param::get("~" + profile_name + "/gripper/limit_upper", joy.gripper.limit_upper);
    ros::param::get("~" + profile_name + "/gripper/max_effort", joy.gripper.goal.command.max_effort);

    /* utils */
    ros::param::get("~" + profile_name + "/arm_mode_btn", joy.utils.joy_btn_arm_mode);
    ros::param::get("~" + profile_name + "/safety_btn", joy.utils.joy_btn_safety);
}

void Armadillo2Teleop::moveGripper()
{
    gripper_client_->sendGoal(joy.gripper.goal);
}

void Armadillo2Teleop::moveArm()
{
    arm_grp_.setJointValueTarget(joy.arm.axes_vals);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (arm_grp_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        arm_grp_.move();
        for (int i=0; i < joy_arm::DOF; i++)
            joy.arm.axes_vals_prev[i] = joy.arm.axes_vals[i];
    }
    else
    {
        ROS_WARN("[armadillo2_teleop]: invalid moveit goal");
        /* if fail, revert to prev values */
        for (int i=0; i < joy_arm::DOF; i++)
            joy.arm.axes_vals[i] = joy.arm.axes_vals_prev[i];
    }
}

void Armadillo2Teleop::resetArm()
{
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if(arm_grp_.plan(start_plan))  //Check if plan is valid
    {
        ROS_INFO("[armadillo2_teleop]: moving arm to start position: %s", joy.arm.start_pos.c_str());
        arm_grp_.execute(start_plan);
    }
    else
        ROS_WARN("[armadillo2_teleop]: failed moving arm to start position: %s", joy.arm.start_pos.c_str());

    arm_grp_.getCurrentState()->copyJointGroupPositions(
            arm_grp_.getCurrentState()->getRobotModel()->getJointModelGroup(arm_grp_.getName()),
            joy.arm.axes_vals);
}


void Armadillo2Teleop::update(const sensor_msgs::Joy::ConstPtr &joy_msg)
{

    //TODO: INIT WITH JOINT INITIAL STATE TO PREVENT MOVEMENT TO 0 ON STARTUP
    //TODO: ALOW STOP-reset FUNTCTION
    //TODO: ADD DEBOUNCER FOR BUTTONS
    if (!joy_msg->buttons[joy.utils.joy_btn_safety])
        return;

    if (!joy_msg->buttons[joy.utils.joy_btn_arm_mode])
    {
        /* drive robot */
        joy.twist.axis_val_angular = joy_msg->axes[joy.twist.joy_axis_angular];
        joy.twist.axis_val_angular *= joy.twist.scale_angular;

        joy.twist.axis_val_linear = joy_msg->axes[joy.twist.joy_axis_linear];
        joy.twist.axis_val_angular *= joy.twist.scale_linear;
        drive();

        /* move torso */
        if (joy_msg->axes[joy.torso.joy_axis_updown] == 1)
        {
            joy.torso.axis_val_updown += joy.torso.increment;
            if (joy.torso.axis_val_updown > joy.torso.limit_upper)
                joy.torso.axis_val_updown = joy.torso.limit_upper;
        }
        else if (joy_msg->axes[joy.torso.joy_axis_updown] == -1)
        {

            joy.torso.axis_val_updown -= joy.torso.increment;
            if (joy.torso.axis_val_updown < joy.torso.limit_lower)
                joy.torso.axis_val_updown = joy.torso.limit_lower;
        }
        if (joy_msg->axes[joy.torso.joy_axis_updown] != 0)
            moveTorso();

        /* move head */
        if (joy_msg->buttons[joy.head.joy_btn_pan_left])
            joy.head.axis_val_pan += joy.head.inc_pan;
        if (joy.head.axis_val_pan < joy.head.limit_lower_pan)
            joy.head.axis_val_pan = joy.head.limit_lower_pan;

        if (joy_msg->buttons[joy.head.joy_btn_pan_right])
            joy.head.axis_val_pan -= joy.head.inc_pan;
        if (joy.head.axis_val_pan > joy.head.limit_upper_pan)
            joy.head.axis_val_pan = joy.head.limit_upper_pan;

        if (joy_msg->buttons[joy.head.joy_btn_tilt_down])
            joy.head.axis_val_tilt += joy.head.inc_tilt;
        if (joy.head.axis_val_tilt < joy.head.limit_lower_tilt)
            joy.head.axis_val_tilt = joy.head.limit_lower_tilt;

        if (joy_msg->buttons[joy.head.joy_btn_tilt_up])
            joy.head.axis_val_tilt -= joy.head.inc_tilt;
        if (joy.head.axis_val_tilt > joy.head.limit_upper_tilt)
            joy.head.axis_val_tilt = joy.head.limit_upper_tilt;

        if (joy_msg->buttons[joy.head.joy_btn_pan_left] ||
            joy_msg->buttons[joy.head.joy_btn_pan_right] ||
            joy_msg->buttons[joy.head.joy_btn_tilt_down] ||
            joy_msg->buttons[joy.head.joy_btn_tilt_up])
            moveHead();
    }
    else
    {
        /* move gripper */
        if (joy_msg->axes[joy.gripper.joy_axis] == 1)
        {
            joy.gripper.goal.command.position += joy.gripper.increment;
            if (joy.gripper.goal.command.position > joy.gripper.limit_upper)
                joy.gripper.goal.command.position = joy.gripper.limit_upper;
        }

        else if (joy_msg->axes[joy.gripper.joy_axis] == -1)
        {
            joy.gripper.goal.command.position -= joy.gripper.increment;
            if (joy.gripper.goal.command.position < joy.gripper.limit_lower)
                joy.gripper.goal.command.position = joy.gripper.limit_lower;
        }
        if (joy_msg->axes[joy.gripper.joy_axis] != 0)
            moveGripper();


        /* move arm */
        joy.arm.axes_vals[joy_arm::INDX_ROTATION1] += joy_msg->axes[joy.arm.joy_axis_rotation1] * joy.arm.increment;
        joy.arm.axes_vals[joy_arm::INDX_SHOULDER1] += joy_msg->axes[joy.arm.joy_axis_shoulder1] * joy.arm.increment;
        joy.arm.axes_vals[joy_arm::INDX_SHOULDER2] -= joy_msg->axes[joy.arm.joy_axis_shoulder2] * joy.arm.increment;
        joy.arm.axes_vals[joy_arm::INDX_ROTATION2] += joy_msg->axes[joy.arm.joy_axis_rotation2] * joy.arm.increment;

        if (joy_msg->buttons[joy.arm.joy_btn_shoulder3_up])
            joy.arm.axes_vals[joy_arm::INDX_SHOULDER3] += joy.arm.increment;
        else if (joy_msg->buttons[joy.arm.joy_btn_shoulder3_down])
            joy.arm.axes_vals[joy_arm::INDX_SHOULDER3] -= joy.arm.increment;

        if (joy_msg->buttons[joy.arm.joy_btn_wrist_cw])
            joy.arm.axes_vals[joy_arm::INDX_WRIST] += joy.arm.increment;
        else if (joy_msg->buttons[joy.arm.joy_btn_wrist_ccw])
            joy.arm.axes_vals[joy_arm::INDX_WRIST] -= joy.arm.increment;

        if (joy_msg->axes[joy.arm.joy_axis_rotation1] ||
            joy_msg->axes[joy.arm.joy_axis_shoulder1] ||
            joy_msg->axes[joy.arm.joy_axis_shoulder2] ||
            joy_msg->axes[joy.arm.joy_axis_rotation2] ||
            joy_msg->buttons[joy.arm.joy_btn_shoulder3_up] ||
            joy_msg->buttons[joy.arm.joy_btn_shoulder3_down] ||
            joy_msg->buttons[joy.arm.joy_btn_wrist_cw] ||
            joy_msg->buttons[joy.arm.joy_btn_wrist_ccw])
            moveArm();

        if (joy_msg->buttons[joy.arm.joy_btn_reset])
            resetArm();
    }

    //fprintf(stderr, "%d\n", joy.head.joy_btn_pan_left);
}


