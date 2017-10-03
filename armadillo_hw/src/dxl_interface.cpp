

#include "dxl_interface.h"

DxlArmInterface::DxlArmInterface()
{

}

DxlArmInterface::PortState DxlArmInterface::openPort(std::string port_name, unsigned int baudrate)
{
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
    port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
    if (port_handler_->openPort())
    {
        if (port_handler_->setBaudRate(baudrate))
        {
            return SUCCESS;
        }
        return PORT_FAIL;
    }
    return BAUDRATE_FAIL;
}

bool DxlArmInterface::ping(DxlMotor &motor)
{
    int result = COMM_TX_FAIL;
    uint8_t error =  0;

    result = packet_handler_->ping(port_handler_,
                                   motor.id,
                                   &motor.model,
                                   &error);

    if (result != COMM_SUCCESS || error != 0)
        return false;
    return true;
}


bool DxlArmInterface::setTorque(const DxlMotor &motor, bool flag)
{
    uint8_t error = 0;
    int result = COMM_TX_FAIL;

    uint16_t torque_register_addr = ADDR_PRO_TORQUE_ENABLE;
    if (motor.model == MODEL_XH430_V350)
        torque_register_addr = ADDR_XH_TORQUE_ENABLE;

    result = packet_handler_->write1ByteTxRx(port_handler_, motor.id, torque_register_addr, flag, &error);

    if (result != COMM_SUCCESS || error != 0)
        return false;
    return true;
}

bool DxlArmInterface::reboot(const DxlMotor &motor)
{

    // Try reboot
    // Dynamixel LED will flicker while it reboots
    uint8_t error = 0;
    int result = COMM_TX_FAIL;
    result = packet_handler_->reboot(port_handler_, motor.id, &error);
    if (result != COMM_SUCCESS || error != 0)
        return false;
    return true;
}

bool DxlArmInterface::broadcastPing(std::vector<uint8_t> result_vec)
{
    int result = COMM_TX_FAIL;
    result = packet_handler_->broadcastPing(port_handler_, result_vec);
    if (result != COMM_SUCCESS)
        return false;
    return true;
}

DxlArmInterface::~DxlArmInterface()
{
    port_handler_->closePort();
}
