//
// Created by armadillo2 on 15/01/18.
//

#include <armadillo2_services/torso.h>

Torso::Torso(ros::NodeHandle &nh)
{
    nh_ = &nh;
    torso_pub_ = nh.advertise<std_msgs::Float64>("/torso_effort_controller/command", 10);
}

bool Torso::command(float elevation)
{
    if (torso_pub_.getNumSubscribers() > 0)
    {
        std_msgs::Float64 cmd_msg;
        cmd_msg.data = elevation;
        torso_pub_.publish(cmd_msg);
        return true;
    }
    return false;
}