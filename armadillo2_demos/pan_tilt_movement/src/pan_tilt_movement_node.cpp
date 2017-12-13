#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_api");
    std::vector<double> q_goal(2);

    if (argc==3) {
        q_goal[0]=atof(argv[1]);
        q_goal[1]=atof(argv[2]);
    }
    else {
        srand (time(NULL));
        q_goal[0]=(double)((rand()%121)-60)*M_PI/180.0;
        q_goal[1]=(double)((rand()%61)-30)*M_PI/180.0;
    }
    ROS_INFO("Pan-Tilt Goal: [%f,%f]",q_goal[0],q_goal[1]);
    ros::NodeHandle nh;
    ros::Publisher pub_controller_command = nh.advertise<trajectory_msgs::JointTrajectory>("pan_tilt_trajectory_controller/command", 2);
    ros::Rate r(1); // 50 hz
    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory traj;
        traj.header.stamp = ros::Time::now();
        traj.joint_names.push_back("head_pan_joint");
        traj.joint_names.push_back("head_tilt_joint");
        traj.points.resize(1);
        traj.points[0].time_from_start = ros::Duration(1.0);
        traj.points[0].positions = q_goal;
        traj.points[0].velocities.push_back(0);
        traj.points[0].velocities.push_back(0);
        pub_controller_command.publish(traj);

        r.sleep();

    }

}
