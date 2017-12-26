#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>

ros::Publisher traj_pub, grp_pos_pub;

void publishTrajectoryMsg(std::vector<double> &head_goal)
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(5.0);
    traj.points[0].positions = head_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    traj_pub.publish(traj);
}

void publishGroupPosMsg(std::vector<double> &head_goal)
{
    std_msgs::Float64MultiArray grp_pos_msg;
    grp_pos_msg.data.push_back((float)head_goal[0]);
    grp_pos_msg.data.push_back((float)head_goal[1]);
    grp_pos_pub.publish(grp_pos_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "center_head_node");
    ros::NodeHandle nh;
    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("pan_tilt_trajectory_controller/command", 5);
    grp_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 5);
    ros::Rate r(1);

    std::vector<double> head_goal(2); //rads
    head_goal[0] = 0;
    head_goal[1] = 0;

    ROS_INFO("[center_head_node]: centering head...");
    ros::Duration timeout(5.0);
    ros::Duration elapsed;
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && elapsed < timeout)
    {
        elapsed = ros::Time::now() - start_time;
        publishGroupPosMsg(head_goal);
        r.sleep();
    }
    ROS_INFO("[center_head_node]: done.");
    return 0;
}
