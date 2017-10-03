

#include "dxl_arm_interface.h"

DxlArmInterface::DxlArmInterface()
{

}

bool DxlArmInterface::openPort(std::string port_name, unsigned int baudrate)
{
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
    port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    if (port_handler_->openPort())
    {
        port_handler_->setBaudRate(baudrate);
        return true;
    }
    return false;
}