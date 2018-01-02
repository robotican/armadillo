#include <armadillo2_teleop/armadillo_teleop.h>


Armadillo2Teleop::Armadillo2Teleop(ros::NodeHandle &nh)
{
    nh_ = &nh;
    twist_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 5);
    torso_pub_ = nh_->advertise<std_msgs::Float64>("torso_effort_controller/command", 5);
}

void Armadillo2Teleop::drive()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = twist.axis_angular * twist.scale_angular;
    twist_msg.linear.x = twist.axis_linear * twist.scale_linear;
    twist_pub_.publish(twist_msg);
}

void Armadillo2Teleop::moveTorso()
{
    std_msgs::Float64 torso_pos;
    torso_pos.data = torso.axis_updown + torso.inc_updown;
    torso_pub_.publish(torso_pos);
}
