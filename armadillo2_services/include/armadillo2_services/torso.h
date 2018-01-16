//
// Created by armadillo2 on 15/01/18.
//

#ifndef ARMADILLO2_SERVICES_TORSO_H
#define ARMADILLO2_SERVICES_TORSO_H

#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <armadillo2_services/joints_state_reader.h>

class Torso
{
private:
    ros::Publisher torso_pub_;
    ros::NodeHandle *nh_;
    const JointStateReader* joints_state_;

public:
    Torso(ros::NodeHandle &nh,
          const JointStateReader &joints_state);
    bool command(float position);
    bool commandCurrentPos();
};


#endif //ARMADILLO2_SERVICES_TORSO_H
