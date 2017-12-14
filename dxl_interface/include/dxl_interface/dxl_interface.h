


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
        uint16_t model = 0;
        float torque_const_a = 0;
        float torque_const_b = 0;
        int cpr = 0;
        double rpm_scale_factor = 0;
        double current_ratio = 0;

        uint16_t pos_read_addr = 0;
        uint16_t vel_read_addr = 0;
        uint16_t current_read_addr = 0;
        uint16_t error_read_addr = 0;

        uint16_t torque_write_addr = 0;
        uint16_t vel_write_addr = 0;
        uint16_t pos_write_addr = 0;

        uint16_t len_present_speed = 0;
        uint16_t len_present_pos = 0;
        uint16_t len_present_curr = 0;
        uint16_t len_goal_speed = 0;
        uint16_t len_goal_pos = 0;
    };

    struct led_color
    {
        uint8_t red,
                green,
                blue;
        led_color(uint8_t red,
                  uint8_t green,
                  uint8_t blue)
        {
            this->red = red;
            this->green = green;
            this->blue = blue;
        }
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

        uint8_t id = 0;
        int direction = 0;
        bool in_torque = false;
        double position = 0;
        double velocity = 0; /* rad/sec */
        double current = 0;
        double command_position = 0;
        double command_velocity = 0.15;
        uint8_t error;

        std::string joint_name;
        InterfaceType interface_type;

        /*******************************************************************************/
        /* dxl api interperate 0 ticks velocity as the highest velocity. */
        /* set set_first_pos_write_to_curr_pos field to true  prevent it */
        /* by setting velocity to the last non-zero value                */
        int32_t prev_non_zero_velocity_ticks = 4; /* 4 ticks is very slow motor speed. */
                                                  /* don't change this feild - it will */
                                                  /* be updated automatically          */
        bool dont_allow_zero_ticks_vel = true;
        /*******************************************************************************/

        /*******************************************************************************/
        /* set set_first_pos_write_to_curr_pos to true        */
        /* to write current position to motors on first write */
        bool first_pos_read = true;/* don't change this feild - it will */
                                   /* be updated automatically          */
        bool set_first_pos_write_to_curr_pos = true;
        /*******************************************************************************/
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
        bool setMotorsLed(std::vector<dxl::motor> &motors,
                          const led_color &color);
    };

}

#endif //ARMADILLO2_HW_ARM_INTERFACE_H
