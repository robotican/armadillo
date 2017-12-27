#include <armadillo2_teleop/armadillo_teleop.h>


Armadillo2Teleop::Armadillo2Teleop(ros::NodeHandle &nh)
{
    nh_ = &nh;
    twist_pub_ = nh_->advertise<geometry_msgs::Twist>("twist_mux/cmd_vel", 5);
}
void Armadillo2Teleop::drive(const twist_joy &twist)
{
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = twist.axis_angular;
    twist_msg.linear.x = twist.axis_linear;
    twist_pub_.publish(twist_msg);
}