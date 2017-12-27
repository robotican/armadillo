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
    ///*
    for (int i=0; i<joy->axes.size(); i++)
        fprintf(stderr, "axes[%i]:%f | ", i, joy->axes[i]);
    fprintf(stderr, "\n");
    //*/

    twist_joy twist;
    twist.axis_angular = joy->axes[twist.joy_axes_angular] * twist.scale_angular;
    twist.axis_linear = joy->axes[twist.joy_axes_linear] * twist.scale_linear;

    armadillo_teleop->drive(twist);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "armadillo2_teleop_node");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
    armadillo_teleop = new Armadillo2Teleop(nh);
    ros::spin();
    delete armadillo_teleop;
    return 0;
}

