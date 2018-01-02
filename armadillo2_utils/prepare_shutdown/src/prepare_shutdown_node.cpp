#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Trigger.h>


ros::Publisher traj_pub;
ros::ServiceServer shutdown_srv;


bool prepareShutdownCB(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res)
{
    /* lower head */
    std::vector<double> head_goal(2);
    head_goal[0] = 0;
    head_goal[1] = 0.7;
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=head_goal[0];
    q_goal[1]=head_goal[1];
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0.1);
    traj.points[0].velocities.push_back(0.1);
    traj_pub.publish(traj);

    /* move arm to driving mode */
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(50);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
    group.setStartStateToCurrentState();
    group.setNamedTarget("driving");
    moveit::planning_interface::MoveGroupInterface::Plan startPosPlan;
    if(group.plan(startPosPlan))
    {
        group.execute(startPosPlan);
        res.message = "shutdown preparation finished";
        res.success = true;
    }
    else
    {
        res.message = "move arm to driving mode failed (planning failed)";
        res.success = false;
    }
    return true;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "prepare_shutdown_node");
    ros::NodeHandle nh;
    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 5);
    shutdown_srv = nh.advertiseService("prepare_shutdown", prepareShutdownCB);


    /* wait for subscribers and publishers to come up */
    ros::Rate loop_rate(1);
    while (traj_pub.getNumSubscribers() <= 0)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("[prepare_shutdown_node]: ready");

    ros::spin();
    return 0;
}

