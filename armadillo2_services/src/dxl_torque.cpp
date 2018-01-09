
#include <armadillo2_services/dxl_torque.h>


DxlTorque::DxlTorque(ros::NodeHandle &nh, const PanTiltMover &head_mover)
{
    nh_ = &nh;
    head_mover_ = &head_mover;
    set_torque_srv_ = nh_->advertiseService("services/dxl_torque", &DxlTorque::setTorqueCB, this);
    set_torque_client_ = nh_->serviceClient<std_srvs::SetBool>("hardware/dxl_torque");
    dxl_traj_pub_ = nh_->advertise<trajectory_msgs::JointTrajectory>("arm_trajectory_controller/command", 5);
    joints_state_sub_ = nh_->subscribe("joint_states", 5, &DxlTorque::jointsUpdateCB, this);
}

bool DxlTorque::setTorque(bool onoff)
{
    std_srvs::SetBool torque_msg;
    torque_msg.request.data = onoff;
    set_torque_client_.call(torque_msg);
    return torque_msg.response.success;
}

bool DxlTorque::setTorqueCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (req.data) //if requested torque on
    {
        if (!arm_.got_state ||
            !head_.got_state)
        {
            res.message = "didn't get arm or head joint state message";
            res.success = false;
            return true;
        }
        commandCurrentDxlPosition();
        ros::Duration(1).sleep();
    }
    if (!set_torque_client_.exists())
    {
        res.message = "hardware/dxl_torque service is not available. are you running in gazebo?";
        res.success = false;
        return true;
    }
    res.success = setTorque(req.data);
    return true;
}

/* command all dxl motors to stay in place, so when */
/* torque comes back on, controller won't send it   */
/* to position before torque off                    */
void DxlTorque::commandCurrentDxlPosition()
{
    trajectory_msgs::JointTrajectory traj_msg;

    traj_msg.joint_names.push_back(ROTATION1_JOINT_NAME);
    traj_msg.joint_names.push_back(ROTATION2_JOINT_NAME);
    traj_msg.joint_names.push_back(SHOULDER1_JOINT_NAME);
    traj_msg.joint_names.push_back(SHOULDER2_JOINT_NAME);
    traj_msg.joint_names.push_back(SHOULDER3_JOINT_NAME);
    traj_msg.joint_names.push_back(WRIST_JOINT_NAME);

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(arm_.rotation1_pos_rad);
    point.positions.push_back(arm_.rotation2_pos_rad);
    point.positions.push_back(arm_.shoulder1_pos_rad);
    point.positions.push_back(arm_.shoulder2_pos_rad);
    point.positions.push_back(arm_.shoulder3_pos_rad);
    point.positions.push_back(arm_.wrist_pos_rad);

    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);
    point.velocities.push_back(0.1);

    point.time_from_start = ros::Duration(1);
    traj_msg.points.push_back(point);
    traj_msg.header.stamp = ros::Time::now();
    dxl_traj_pub_.publish(traj_msg);

    head_mover_->publishTrajectoryMsg(head_.pan_pos_rad, head_.tilt_pos_rad);
}

/* save joints updated state */
void DxlTorque::jointsUpdateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    arm_.got_state = true;
    arm_.rotation1_pos_rad = msg->position[ROTATION1_JOINT_INDX];
    arm_.rotation2_pos_rad = msg->position[ROTATION2_JOINT_INDX];
    arm_.shoulder1_pos_rad = msg->position[SHOULDER1_JOINT_INDX];
    arm_.shoulder2_pos_rad = msg->position[SHOULDER2_JOINT_INDX];
    arm_.shoulder3_pos_rad = msg->position[SHOULDER3_JOINT_INDX];
    arm_.wrist_pos_rad = msg->position[WRIST_JOINT_INDX];

    head_.got_state = true;
    head_.pan_pos_rad = msg->position[PAN_JOINT_INDX];
    head_.tilt_pos_rad = msg->position[TILT_JOINT_INDX];
}