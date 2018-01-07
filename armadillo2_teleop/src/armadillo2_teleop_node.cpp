#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armadillo2_teleop/armadillo_teleop.h>

Armadillo2Teleop *armadillo_teleop;

void printAxes()
{

}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    /* print axes for testing */
    /*for (int i=0; i<joy->axes.size(); i++)
        fprintf(stderr, "axes[%i]:%f | ", i, joy->axes[i]);
    fprintf(stderr, "\n");*/
    /*for (int i=0; i<joy->buttons.size(); i++)
        fprintf(stderr, "axes[%i]:%f | ", i, joy->buttons[i]);
    fprintf(stderr, "\n");*/

    armadillo_teleop->update(joy);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "armadillo2_teleop_node");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
    armadillo_teleop = new Armadillo2Teleop(nh);
    std::string profile;
    if (!ros::param::get("~profile", profile))
    {
        ROS_ERROR("[armadillo2_teleop_node]: couldn't find profile param. shutting down...");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("[armadillo2_teleop_node]: loading %s profile...", profile.c_str());
    armadillo_teleop->loadProfile(profile);
    ROS_INFO("[armadillo2_teleop_node]: ready to dance");
    ros::spin();
    delete armadillo_teleop;
    return 0;
}

