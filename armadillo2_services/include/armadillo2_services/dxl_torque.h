//
// Created by armadillo2 on 08/01/18.
//

#ifndef ARMADILLO2_SERVICES_DXL_TORQUE_H
#define ARMADILLO2_SERVICES_DXL_TORQUE_H

#include <ros/ros.h>

class DxlTorque
{
private:
    ros::NodeHandle* nh_;
public:
    DxlTorque(ros::NodeHandle& nh);
    bool set(bool onoff) const;

};


#endif //ARMADILLO2_SERVICES_DXL_TORQUE_H
