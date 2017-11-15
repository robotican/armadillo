


#ifndef ARMADILLO2_HW_ARM_INTERFACE_H
#define ARMADILLO2_HW_ARM_INTERFACE_H

#include <iostream>
#include <stdint.h>
#include <cmath>
#include <dynamixel_sdk/dynamixel_sdk.h>

#define DXL_PROTOCOL1 1.0
#define DXL_PROTOCOL2 2.0
#define DXL_ERR_LEN 1

namespace dxl
{
    struct spec
    {
        std::string name;
        uint16_t model;
        float torque_const_a;
        float torque_const_b;
        int cpr;
        double rpm_scale_factor;
        double current_ratio;

        uint16_t pos_read_addr;
        uint16_t vel_read_addr;
        uint16_t current_read_addr;
        uint16_t error_read_addr;

        uint16_t torque_write_addr;
        uint16_t vel_write_addr;
        uint16_t pos_write_addr;

        uint16_t len_present_speed;
        uint16_t len_present_pos;
        uint16_t len_present_curr;
        uint16_t len_goal_speed;
        uint16_t len_goal_pos;
    };

    struct motor
    {
        enum InterfaceType
        {
            POS,
            POS_VEL
        };

        static InterfaceType stringToInterfaceType(std::string type)
        {
            if (type == "Position")
                return POS;
            if (type == "PosVel")
                return POS_VEL;
        }

        dxl::spec spec;

        uint8_t id;
        bool in_torque;
        double position;
        double velocity; //rad/sec
        double current;
        double command_position;
        double command_velocity;
        uint8_t error;

        std::string joint_name;
        InterfaceType interface_type;

        /* dxl api interperate 0 velocity as the highest velocity. */
        /* this field prevent it by setting velocity to the last   */
        /* non-zero value                                          */
        double pre_vel; //rad/sec

        bool first_pos_read = true;

    };

    namespace convertions
    {
        double ticks2rads(int32_t ticks, struct motor &motor, float protocol);
        int32_t rads2ticks(double rads, struct motor &motor, float protocol);
        int32_t rad_s2ticks_s(double rads, struct motor &motor, float protocol);
        double ticks_s2rad_s(int32_t ticks, struct motor &motor, float protocol);
    }

    class DxlInterface
    {

    private:
        dynamixel::PacketHandler *pkt_handler_;
        dynamixel::PortHandler *port_handler_;
        float protocol_;

        bool loadProtocol(uint16_t protocol);

    public:

        enum PortState
        {
            PORT_FAIL,
            BAUDRATE_FAIL,
            INVALID_PROTOCOL,
            SUCCESS
        };

        DxlInterface();
        ~DxlInterface();
        PortState openPort(std::string port_name,
                           unsigned int baudrate,
                           float protocol);
        bool ping (motor & motor);
        bool setTorque(motor &motor, bool flag);
        bool bulkWriteVelocity(std::vector<motor> & motors);
        bool bulkWritePosition(std::vector<motor> & motors);
        bool readMotorsPos(std::vector<motor> & motors);
        bool readMotorsVel(std::vector<motor> & motors);
        bool readMotorsLoad(std::vector<motor> &motors);
        bool readMotorsError(std::vector<motor> & motors);
        bool reboot(const motor &motor);
        bool broadcastPing(std::vector<uint8_t> result_vec, uint16_t protocol);
    };

}

#endif //ARMADILLO2_HW_ARM_INTERFACE_H
