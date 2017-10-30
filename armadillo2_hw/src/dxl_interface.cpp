

#include "dxl_interface.h"

namespace dxl
{

/***** DXL INTERFACE *****/

    DxlInterface::DxlInterface()
    {
    }

    DxlInterface::PortState DxlInterface::openPort(std::string port_name, unsigned int baudrate)
    {
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
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

    bool DxlInterface::ping(motor &motor)
    {
        int result = COMM_TX_FAIL;
        uint8_t error =  0;

        result = packet_handler_->ping(port_handler_,
                                       motor.id,
                                       &(motor.spec.model),
                                       &error);

        if (result != COMM_SUCCESS || error != 0)
            return false;
        return true;
    }


    bool DxlInterface::setTorque(const motor &motor, bool flag)
    {
        uint8_t error = 0;
        int result = COMM_TX_FAIL;

        uint16_t torque_register_addr = ADDR_PRO_TORQUE_ENABLE;
        if (motor.spec.model == MODEL_XH430_V350)
            torque_register_addr = ADDR_XH_TORQUE_ENABLE;

        result = packet_handler_->write1ByteTxRx(port_handler_, motor.id, torque_register_addr, flag, &error);

        if (result != COMM_SUCCESS || error != 0)
            return false;
        return true;
    }

    bool DxlInterface::reboot(const motor &motor)
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

    bool DxlInterface::broadcastPing(std::vector<uint8_t> result_vec)
    {
        int result = COMM_TX_FAIL;
        result = packet_handler_->broadcastPing(port_handler_, result_vec);
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
        dynamixel::GroupBulkRead bulk_read(port_handler_, packet_handler_);
        for (motor &motor : motors)
        {
            uint16_t addr = ADDR_PRO_PRESENT_POSITION;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PRESENT_POSITION;
            bool read_success = bulk_read.addParam(motor.id, addr, LEN_PRO_PRESENT_POSITION);
            if (!read_success)
                return false;
        }

        int comm_result = bulk_read.txRxPacket();
        if (comm_result != COMM_SUCCESS)
            //packet_handler_->printTxRxResult(comm_result);
            return false;

        for (motor &motor : motors)
        {
            uint16_t addr = ADDR_PRO_PRESENT_POSITION;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PRESENT_POSITION;
            bool getdata_result = bulk_read.isAvailable(motor.id, addr, LEN_PRO_PRESENT_POSITION);
            if (!getdata_result)
                return false;

            uint32_t ticks = bulk_read.getData(motor.id, addr, LEN_PRO_PRESENT_POSITION);
            motor.position =  convertions::ticks2rads(ticks, motor);
        }
        return true;
    }

    bool DxlInterface::readMotorsVel(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, packet_handler_);
        for (motor &motor : motors)
        {
            uint16_t addr = ADDR_PRO_PRESENT_SPEED;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PRESENT_SPEED;
            bool read_success = bulk_read.addParam(motor.id, addr, LEN_PRO_PRESENT_SPEED);
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
            uint16_t addr = ADDR_PRO_PRESENT_SPEED;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PRESENT_SPEED;
            bool getdata_result = bulk_read.isAvailable(motor.id, addr, LEN_PRO_PRESENT_SPEED);
            if (!getdata_result)
                return false;

            uint32_t ticks_per_sec = bulk_read.getData(motor.id, addr, LEN_PRO_PRESENT_SPEED);
            motor.velocity = convertions::ticks_s2rad_s(ticks_per_sec, motor);
        }
        return true;
    }

    bool DxlInterface::readMotorsError(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, packet_handler_);
        for (motor &motor : motors)
        {
            uint16_t addr = ADDR_PRO_HARDWARE_ERROR;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_HARDWARE_ERROR;
            bool read_success = bulk_read.addParam(motor.id, addr, LEN_PRO_PRESENT_ERROR);
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
            uint16_t addr = ADDR_PRO_HARDWARE_ERROR;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_HARDWARE_ERROR;
            bool getdata_result = bulk_read.isAvailable(motor.id, addr, LEN_PRO_PRESENT_ERROR);
            if (!getdata_result)
                return false;

            motor.error = bulk_read.getData(motor.id, addr, LEN_PRO_PRESENT_ERROR);
            if (motor.error)
                return false;
        }
        return true;
    }

    bool DxlInterface::readMotorosLoad(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkRead bulk_read(port_handler_, packet_handler_);
        for (motor &motor : motors)
        {
            uint16_t addr = ADDR_PRO_PRESENT_CURRENT;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PRESENT_CURRENT;
            bool read_success = bulk_read.addParam(motor.id, addr, LEN_PRO_PRESENT_CURRENT);
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
            uint16_t addr = ADDR_PRO_PRESENT_CURRENT;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PRESENT_CURRENT; //
            bool getdata_result = bulk_read.isAvailable(motor.id, addr, LEN_PRO_PRESENT_CURRENT);
            if (!getdata_result)
                return false;


            motor.current = (int16_t)bulk_read.getData(motor.id, addr, LEN_PRO_PRESENT_CURRENT) ;

            switch (motor.spec.model)
            {
                case MODEL_H54_100_S500_R || MODEL_H54_200_S500_R:
                    motor.current *= 33000.0 / 2048.0 / 1000.0;
                    break;
                case MODEL_H42_20_S300_R:
                    motor.current *= 8250.0 / 2048.0 / 1000.0;
                    break;
                case MODEL_XH430_V350:
                    motor.current *= 2.69/1000.0;
                    break;
            }
            /*   if (motor.id ==6)/////////////////////////////////////////////////////
               {
                   printf("CURRENT: %f\n", motor.current);/////////////////////////////////////////////////////////////////
                   printf("torque_const_a: %f, torque_const_b: %f\n", motor.spec.torque_const_a, motor.spec.torque_const_b);
               }
               motor.effort = motor.spec.torque_const_a * motor.current + motor.spec.torque_const_b;
               if (motor.id ==6)
                   printf("EFFORT: %f\n", motor.effort);/////////////////////////////////////////////////////////////////*/
        }
        return true;
    }

    bool DxlInterface::bulkWriteVelocity(std::vector<dxl::motor> &motors)
    {
        dynamixel::GroupBulkWrite bulk_write(port_handler_, packet_handler_);

        for (motor &motor : motors)
        {
            bool addparam_success = false;
            uint16_t addr = ADDR_PRO_GOAL_SPEED;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_PROFILE_VELOCITY;
            if (motor.command_velocity == 0)
                motor.command_velocity = motor.pre_vel;
            else
                motor.pre_vel = motor.command_velocity;

            int32_t motor_vel = convertions::rad_s2ticks_s(motor.command_velocity, motor);

            addparam_success = bulk_write.addParam(motor.id, addr, LEN_PRO_PRESENT_SPEED, (uint8_t*)&motor_vel);
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
        dynamixel::GroupBulkWrite bulk_write(port_handler_, packet_handler_);

        for (motor &motor : motors)
        {
            bool addparam_success = false;
            uint16_t addr = ADDR_PRO_GOAL_POSITION;
            if (motor.spec.model == MODEL_XH430_V350)
                addr = ADDR_XH_GOAL_POSITION;
            int32_t motor_pos = convertions::rads2ticks(motor.command_position, motor);
            addparam_success = bulk_write.addParam(motor.id, addr, LEN_PRO_PRESENT_POSITION, (uint8_t*)&motor_pos);
            if (!addparam_success)
                return false;
        }

        int8_t comm_result_ = bulk_write.txPacket();

        if (comm_result_ != COMM_SUCCESS)
            return false;

        bulk_write.clearParam();
        return true;
    }

    /**** DXL MATH *****/
    double convertions::ticks2rads(int32_t ticks, struct motor &motor)
    {
        if (motor.protocol_ver == 2.0)
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

    int32_t convertions::rads2ticks(double rads, struct motor &motor)
    {

        if (motor.protocol_ver == 2.0) {
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
    int32_t convertions::rad_s2ticks_s(double rads, struct motor &motor)
    {
        if (motor.protocol_ver == 2.0)
            return static_cast<int32_t >(rads / 2.0 / M_PI * 60.0 / motor.spec.rpm_scale_factor);
        else
            return static_cast<int32_t >(83.49f * (rads)-0.564f);
    }

    /* ticks per sec to rads per sec */
    double convertions::ticks_s2rad_s(int32_t ticks, struct motor &motor)
    {
        if (motor.protocol_ver == 2.0)
            return ((double)ticks) * 2.0 * M_PI / 60.0 / motor.spec.rpm_scale_factor;
        else
            return (100.0f / 8349.0f) * ((double)ticks) + (94.0f / 13915.0f);
    }


}
