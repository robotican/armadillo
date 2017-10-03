
#include "armadillo_hw.h"



void ArmadilloHW::openArmPort(std::string port_name, unsigned int baudrate)
{
    DxlInterface::PortState port_state = dxl_.openPort(port_name, baudrate);
    switch (port_state)
    {
        case DxlInterface::PortState::PORT_FAIL:
            ROS_ERROR_NAMED("armadillo_hw", "open arm port %s failed. shutting down...", port_name);
            ros::shutdown();
            break;
        case DxlInterface::PortState::BAUDRATE_FAIL:
            ROS_ERROR_NAMED("armadillo_hw", "setting arm baudrate to %u failed. shutting down...", baudrate);
            ros::shutdown();
            break;
        case DxlInterface::PortState::SUCCESS:
            ROS_ERROR_NAMED("armadillo_hw", "arm port opened successfully\nport name: %s\nbaudrate: %u", port_name, baudrate);
    }
}

ros::Duration ArmadilloHW::getPeriod()
{
    ros::Time now = getTime();
    ros::Duration period = now - prev_time_;
    prev_time_ = now;
    return period;
}




//if opened port
//        ROS_INFO_NAMED(MY_NAME, "Port: %s , Baudrate: %u opened successfully", port_name, baudrate);
//if not throw error and ros shutdown