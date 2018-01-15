//
// Created by armadillo2 on 15/01/18.
//

#ifndef ARMADILLO2_SERVICES_TORSO_H
#define ARMADILLO2_SERVICES_TORSO_H

#include <std_msgs/Float64.h>
#include <ros/ros.h>

class Torso
{
private:
    ros::Publisher torso_pub_;
    ros::NodeHandle *nh_;

public:
    Torso(ros::NodeHandle &nh);
    float getElevation();
    bool command(float elevation);
};


#endif //ARMADILLO2_SERVICES_TORSO_H
