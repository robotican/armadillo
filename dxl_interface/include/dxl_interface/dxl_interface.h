


#ifndef ARMADILLO2_HW_ARM_INTERFACE_H
#define ARMADILLO2_HW_ARM_INTERFACE_H

// Control table address FOR MX-28
#define ADDR_MX_MODEL_NUM               30
#define ADDR_MX_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PROFILE_VELOCITY        112
#define ADDR_MX_PRESENT_POSITION        132
#define ADDR_MX_PRESENT_SPEED           128
#define ADDR_MX_PRESENT_LOAD            126
#define ADDR_MX_PRESENT_TEMPERATURE     146
#define ADDR_MX_MOVING                  122
#define ADDR_MX_HARDWARE_ERROR          70

// Control table address FOR Pro
#define ADDR_PRO_MODEL_NUM               0
#define ADDR_PRO_TORQUE_ENABLE           562                  // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION           596
#define ADDR_PRO_GOAL_SPEED              600
#define ADDR_PRO_GOAL_ACCELERATION       606
#define ADDR_PRO_PRESENT_POSITION        611
#define ADDR_PRO_PRESENT_SPEED           615
#define ADDR_PRO_PRESENT_CURRENT         621
#define ADDR_PRO_PRESENT_TEMPERATURE     43
#define ADDR_PRO_MOVING                  46
#define ADDR_PRO_HARDWARE_ERROR          892


// Control table address FOR XH
#define ADDR_XH_MODEL_NUM               0
#define ADDR_XH_TORQUE_ENABLE           64                  // Control table address is different in Dynamixel model
#define ADDR_XH_GOAL_POSITION           116
#define ADDR_XH_GOAL_SPEED              104
#define ADDR_XH_VELOCITY_LIMIT          44
#define ADDR_XH_PROFILE_VELOCITY       112
#define ADDR_XH_GOAL_ACCELERATION       40
#define ADDR_XH_PRESENT_POSITION        132
#define ADDR_XH_PRESENT_SPEED           128
#define ADDR_XH_PRESENT_CURRENT         126
#define ADDR_XH_PRESENT_TEMPERATURE     146
#define ADDR_XH_MOVING                  123
#define ADDR_XH_HARDWARE_ERROR          70

#define LEN_PRESENT_POSITION 4
#define LEN_PRESENT_ERROR 1
#define LEN_PRESENT_SPEED 4
#define LEN_PRESENT_CURRENT 2


#define PROTOCOL_VERSION1 2.0
#define PROTOCOL_VERSION2 2.0

#include <iostream>
#include <stdint.h>
#include <cmath>
#include <dynamixel_sdk/dynamixel_sdk.h>

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
        uint16_t current_write_addr;
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

        float protocol_ver;
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
        double ticks2rads(int32_t ticks, struct motor &motor);
        int32_t rads2ticks(double rads, struct motor &motor);
        int32_t rad_s2ticks_s(double rads, struct motor &motor);
        double ticks_s2rad_s(int32_t ticks, struct motor &motor);
    }

    class DxlInterface
    {

    private:
        dynamixel::PacketHandler *packet_handler_;
        dynamixel::PortHandler *port_handler_;

    public:

        enum PortState
        {
            PORT_FAIL,
            BAUDRATE_FAIL,
            SUCCESS
        };

        DxlInterface();
        ~DxlInterface();
        PortState openPort(std::string port_name, unsigned int baudrate);
        bool ping (motor & motor);
        bool setTorque(motor &motor, bool flag);
        bool bulkWriteVelocity(std::vector<motor> & motors);
        bool bulkWritePosition(std::vector<motor> & motors);
        bool readMotorsPos(std::vector<motor> & motors);
        bool readMotorsVel(std::vector<motor> & motors);
        bool readMotorsLoad(std::vector<motor> &motors);
        bool readMotorsError(std::vector<motor> & motors);
        bool reboot(const motor &motor);
        bool broadcastPing(std::vector<uint8_t> result_vec);
    };

}

#endif //ARMADILLO2_HW_ARM_INTERFACE_H
