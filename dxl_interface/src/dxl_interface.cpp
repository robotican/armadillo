

#include "dxl_interface/dxl_interface.h"

namespace dxl
{

/***** DXL INTERFACE *****/

    DxlInterface::DxlInterface()
    {

    }

    DxlInterface::PortState DxlInterface::openPort(std::string port_name,
                                                   unsigned int baudrate,
                                                   float protocol)
    {
        protocol_ = protocol;
        if (!loadProtocol(protocol_))
            return INVALID_PROTOCOL;
        port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());
        if (port_handler_->openPort())
        {
            if (port_handler_->setBaudRate(baudrate))
            {
                return SUCCESS;
            }
            return BAUDRATE_FAIL;
        }
        return PORT_FAIL;
    }

    bool DxlInterface::loadProtocol(uint16_t protocol)
    {
        if (protocol == DXL_PROTOCOL1)
        {
            pkt_handler_ = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL1);
            return true;
        }
        if (protocol == DXL_PROTOCOL2)
        {
            pkt_handler_ = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL2);
            return true;
        }
        return true;
    }

    /* ping motor, if success - motor model will be filled */
    bool DxlInterface::ping(dxl::motor &motor)
    {
        int result = COMM_TX_FAIL;
        uint8_t error =  0;

        result = pkt_handler_->ping(port_handler_,
                                       motor.id,
                                       &(motor.spec.model),
                                       &error);

        if (result != COMM_SUCCESS || error != 0)
            return false;
        return true;
    }


    bool DxlInterface::setTorque(dxl::motor &motor, bool flag)
    {
        uint8_t error = 0;
        int result = COMM_TX_FAIL;

        result = pkt_handler_->write1ByteTxRx(port_handler_,
                                                 motor.id,
                                                 motor.spec.torque_write_addr,
                                                 flag,
                                                 &error);

        if (result != COMM_SUCCESS || error != 0)
            return false;

        motor.in_torque = flag;
        return true;
    }

    bool DxlInterface::reboot(const motor &motor)
    {

        // Try reboot
        // Dynamixel LED will flicker while it reboots
        uint8_t error = 0;
        int result = COMM_TX_FAIL;

        result = pkt_handler_->reboot(port_handler_, motor.id, &error);
        if (result != COMM_SUCCESS || error != 0)
            return false;
        return true;
    }


    bool DxlInterface::broadcastPing(std::vector<uint8_t> result_vec, uint16_t protocol)
    {
        int result = COMM_TX_FAIL;

        result = pkt_handler_->broadcastPing(port_handler_, result_vec);
        if (result != COMM_SUCCESS)
            return false;
        return true;
    }

    DxlInterface::~DxlInterface()
    {
        port_handler_->closePort();
    }

    bool DxlInterface::readMotorsPos(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.pos_read_addr,
                                                   motor.spec.len_present_pos);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
            //packet_handler_->printTxRxResult(comm_result);
            return false;

        for (motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.pos_read_addr,
                                                        motor.spec.len_present_pos);
            if (!getdata_result)
                return false;

            uint32_t ticks = bulk_read.getData(motor.id,
                                               motor.spec.pos_read_addr,
                                               motor.spec.len_present_pos);
            motor.position =  convertions::ticks2rads(ticks, motor, protocol_) * motor.direction;

            if (motor.first_pos_read)
            {
                motor.first_pos_read = false;
                motor.command_position = motor.position;
            }
        }
        return true;
    }

    bool DxlInterface::readMotorsVel(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.vel_read_addr,
                                                   motor.spec.len_present_speed);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
        {
            //packet_handler_->printTxRxResult(comm_result);
            return false;
        }

        for (motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.vel_read_addr,
                                                        motor.spec.len_present_speed);
            if (!getdata_result)
                return false;

            uint32_t ticks_per_sec = bulk_read.getData(motor.id,
                                                       motor.spec.vel_read_addr,
                                                       motor.spec.len_present_speed);
            motor.velocity = convertions::ticks_s2rad_s(ticks_per_sec, motor, protocol_);
        }

        return true;
    }

    bool DxlInterface::readMotorsError(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.error_read_addr,
                                                   DXL_ERR_LEN);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
        {
            //packet_handler_->printTxRxResult(comm_result);
            return false;
        }

        for (motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.error_read_addr,
                                                        DXL_ERR_LEN);
            if (!getdata_result)
                return false;

            motor.error = bulk_read.getData(motor.id,
                                            motor.spec.error_read_addr,
                                            DXL_ERR_LEN);
        }
        return true;
    }

    bool DxlInterface::readMotorsLoad(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, pkt_handler_);
        for (motor &motor : motors)
        {
            bool read_success = bulk_read.addParam(motor.id,
                                                   motor.spec.current_read_addr,
                                                   motor.spec.len_present_curr);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
        {
            pkt_handler_->printTxRxResult(comm_result);
            return false;
        }

        for (motor &motor : motors)
        {
            bool getdata_result = bulk_read.isAvailable(motor.id,
                                                        motor.spec.current_read_addr,
                                                        motor.spec.len_present_curr);
            if (!getdata_result)
                return false;


            motor.current = (int16_t)bulk_read.getData(motor.id,
                                                       motor.spec.current_read_addr,
                                                       motor.spec.len_present_curr);

            motor.current *= motor.spec.current_ratio;
        }
        return true;
    }

    bool DxlInterface::bulkWriteVelocity(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkWrite bulk_write(port_handler_, pkt_handler_);

        for (motor &motor : motors)
        {
            bool addparam_success = false;
            if (motor.command_velocity == 0)
                motor.command_velocity = motor.pre_vel;
            else
                motor.pre_vel = motor.command_velocity;

            int32_t motor_vel = convertions::rad_s2ticks_s(motor.command_velocity, motor, protocol_);
            addparam_success = bulk_write.addParam(motor.id,
                                                   motor.spec.vel_write_addr,
                                                   motor.spec.len_goal_speed,
                                                   (uint8_t*)&motor_vel);
            if (!addparam_success)
                return false;
        }

        int8_t comm_result_ = bulk_write.txPacket();

        if (comm_result_ != COMM_SUCCESS)
            return false;

        bulk_write.clearParam();
        return true;
    }

    bool DxlInterface::bulkWritePosition(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkWrite bulk_write(port_handler_, pkt_handler_);

        for (motor &motor : motors)
        {
            bool addparam_success = false;
            int32_t motor_pos = convertions::rads2ticks(motor.command_position * motor.direction, motor, protocol_);
            addparam_success = bulk_write.addParam(motor.id,
                                                   motor.spec.pos_write_addr,
                                                   motor.spec.len_goal_pos,
                                                   (uint8_t*)&motor_pos);
            if (!addparam_success)
                return false;
        }

        int8_t comm_result_ = bulk_write.txPacket();

        if (comm_result_ != COMM_SUCCESS)
            return false;

        bulk_write.clearParam();
        return true;
    }

    bool DxlInterface::setMotorsLed(std::vector<dxl::motor> &motors,
                                    const led_color &color)
    {
        /* todo: implement. reason for not implementing so far: some motor models
         * have only red led, and some have rgb led. red led looks like warning */
    }

    /**** DXL MATH *****/
    double convertions::ticks2rads(int32_t ticks, struct motor &motor, float protocol)
    {
        if (protocol == DXL_PROTOCOL2)
        {
            if (motor.spec.model==1040)
            {
                const double FromTicks = 1.0 / (motor.spec.cpr / 2.0);
                return static_cast<double>(M_PI-(ticks) * FromTicks * M_PI);
            }
            else if (motor.spec.model==30)
            {
                double cprDev2 = motor.spec.cpr / 2.0f;
                return (static_cast<double>(ticks) - cprDev2) * M_PI / cprDev2;
            }
            else
            {
                const double FromTicks = 1.0 / (motor.spec.cpr / 2.0);
                return static_cast<double>((ticks) * FromTicks * M_PI);
            }
        }
        else
        {
            double cprDev2 = motor.spec.cpr / 2.0f;
            return (static_cast<double>(ticks) - cprDev2) * M_PI / cprDev2;
        }
    }

    int32_t convertions::rads2ticks(double rads, struct motor &motor, float protocol)
    {

        if (protocol == DXL_PROTOCOL2) {
            if (motor.spec.model==1040) {
                return static_cast<int32_t>(round((-rads *180.0/ M_PI+180.0)/ 0.088));
            }
            else if (motor.spec.model==30) {
                double cprDev2 = motor.spec.cpr / 2.0f;
                return static_cast<int32_t>(round(cprDev2 + (rads * cprDev2 / M_PI)));
            }
            else {
                double cprDev2 = motor.spec.cpr / 2.0f;
                return static_cast<int32_t>(round((rads / M_PI) * cprDev2));
            }
        }
        else {
            double cprDev2 = motor.spec.cpr / 2.0f;
            return static_cast<int32_t>(round(cprDev2 + (rads * cprDev2 / M_PI)));
        }
    }

    /* rads per sec to ticks per sec */
    int32_t convertions::rad_s2ticks_s(double rads, struct motor &motor, float protocol)
    {
        if (protocol == DXL_PROTOCOL2)
            return static_cast<int32_t >(rads / 2.0 / M_PI * 60.0 / motor.spec.rpm_scale_factor);
        else
            return static_cast<int32_t >(83.49f * (rads)-0.564f);
    }

    /* ticks per sec to rads per sec */
    double convertions::ticks_s2rad_s(int32_t ticks, struct motor &motor, float protocol)
    {
        if (protocol == DXL_PROTOCOL2)
            return ((double)ticks) * 2.0 * M_PI / 60.0 / motor.spec.rpm_scale_factor;
        else
            return (100.0f / 8349.0f) * ((double)ticks) + (94.0f / 13915.0f);
    }


}
