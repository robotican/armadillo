
#include <armadillo2_services/pan_tilt_mover.h>

PanTiltMover::PanTiltMover(ros::NodeHandle &nh)
{
    nh_ = &nh;
    traj_pub_ = nh_->advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 5);
    //grp_pos_pub_ = nh_->advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 5);
    mover_srv_ = nh_->advertiseService("services/pan_tilt_mover", &PanTiltMover::moveHeadCB, this);
}

bool PanTiltMover::moveHeadCB(armadillo2_msgs::PanTilt::Request &req,
                              armadillo2_msgs::PanTilt::Response &res)
{
    return publishTrajectoryMsg(req.pan, req.tilt);
    //return publishGroupPosMsg(req.pan, req.tilt);
}

bool PanTiltMover::centerHead() const
{
    return publishTrajectoryMsg(0, 0);
}

bool PanTiltMover::publishTrajectoryMsg(float pan, float tilt) const
{
    if (traj_pub_.getNumSubscribers()<=0)
        return false;
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0] = pan;
    q_goal[1] = tilt;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    traj_pub_.publish(traj);
    return true;
}

bool PanTiltMover::publishGroupPosMsg(float pan, float tilt) const
{
    if (grp_pos_pub_.getNumSubscribers()<=0)
        return false;
    std_msgs::Float64MultiArray grp_pos_msg;
    grp_pos_msg.data.push_back(pan);
    grp_pos_msg.data.push_back(tilt);
    grp_pos_pub_.publish(grp_pos_msg);
    return true;
}
