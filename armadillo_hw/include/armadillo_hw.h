
#ifndef ARMADILLO_HW_ARMADILLO_HW_H
#define ARMADILLO_HW_ARMADILLO_HW_H

#include "dxl_interface.h"
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

struct armadillo_arm
{
    uint8_t num_of_motors; //degree of freedom / num of motors

};

class ArmadilloHW : public hardware_interface::RobotHW
{
private:

    ros::Time prev_time_;
    DxlInterface dxl_;

    void registerInterfaces();
    void openArmPort(std::string port_name, unsigned int baudrate);

public:

    ArmadilloHW(ros::NodeHandle &nh);
    void read();
    void write();
    static ros::Time getTime() { return ros::Time::now(); }
    ros::Duration getPeriod();

};

#endif //ARMADILLO_HW_ARMADILLO_HW_H
