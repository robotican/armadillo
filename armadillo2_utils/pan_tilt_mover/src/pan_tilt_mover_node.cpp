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
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=head_goal[0];
    q_goal[1]=head_goal[1];
    traj.points[0].positions=q_goal;
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
    ros::init(argc, argv, "pan_tilt_api");
    ros::NodeHandle nh;
    traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 5);
  //  grp_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 5);
 //   ros::Rate r(15); //

    std::vector<double> head_goal(2); //rads
    float pan_angle = 0, tilt_angle = 0;
    float timeout = 0; //secs
    bool gen_random_angles = false;

    ros::param::get("~pan_angle", pan_angle);
    ros::param::get("~tilt_angle", tilt_angle);
    ros::param::get("~random", gen_random_angles);
    ros::param::get("~timeout", timeout);


    if (gen_random_angles)
    {
        ROS_INFO("[move_pan_tilt_node]: generating random goal...");
        srand (time(NULL));
        pan_angle = (float)((rand()%91)-45);
        tilt_angle = (float)((rand()%81)-50);
        head_goal[0] = pan_angle * (M_PI / 180.0);
        head_goal[1] = tilt_angle * (M_PI / 180.0);
    }
    else
    {
        head_goal[0] = pan_angle * (M_PI / 180);
        head_goal[1] = tilt_angle * (M_PI / 180);
    }

   

    ros::Duration max_period(timeout);
    ros::Duration elapsed;
    ros::Time start_time = ros::Time::now();
//r.sleep();
 ROS_INFO("[move_pan_tilt_node]: waiting for subscriber");
    /* if timeout is 0, run forever. else, run until timeout */
    

   while (ros::ok())
    {
while (traj_pub.getNumSubscribers()<=0) {
 ros::spinOnce();
ros::Duration(3).sleep();
}
 ROS_INFO("[move_pan_tilt_node]: sending pan-tilt goal [%f,%f](degrees)", pan_angle, tilt_angle);
       // elapsed = ros::Time::now() - start_time;
        // publishGroupPosMsg(head_goal);
        publishTrajectoryMsg(head_goal);
        ros::spinOnce();
       // r.sleep();
        ros::shutdown();
    }

    ROS_INFO("[move_pan_tilt_node]: done");
}
