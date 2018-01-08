
#include <armadillo2_services/dxl_torque.h>


DxlTorque::DxlTorque(ros::NodeHandle &nh)
{
    nh_ = &nh;
}

bool DxlTorque::set(bool onoff) const
{
    //TODO: call torque off service in hw
    return true;
}