


#ifndef ARMADILLO_HW_ARM_INTERFACE_H
#define ARMADILLO_HW_ARM_INTERFACE_H

// Control table address FOR MX-28
#define ADDR_MX_MODEL_NUM               0
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_GOAL_SPEED              32
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_PRESENT_SPEED           38
#define ADDR_MX_PRESENT_LOAD            40
#define ADDR_MX_PRESENT_TEMPERATURE     43
#define ADDR_MX_MOVING                  46
#define ADDR_MX_GOAL_ACCELERATION       73

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

/* This library supports only protocol 2.0, */
/* motors using protocol 1.0 will not work  */
#define PROTOCOL_VERSION2               2.0

/* This library supports the following dxl */
/* models defined here: (for more models,  */
/* define them here, and edit code to      */
/* support them accordingly                */
#define MODEL_XH430_V350 1040

#include <iostream>
#include <stdint.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

struct dxl_motor
{
    enum InterfaceType
    {
        POS,
        POS_VEL
    };

    uint8_t id;
    uint16_t model;
    bool torque;
    double position;
    double velocity;
    double current;
    double effort;
    double command_position;
    double command_velocity;
    uint8_t error;
    int cpr;
    double rpm_scale_factor;
    float protocol_ver;
    std::string joint_name;
    InterfaceType interface_type;
    float torque_constant_a;
    float torque_constant_b;
};

class DxlMath
{
public:
    double static ticksToRads(int32_t ticks, dxl_motor &motor);
    int32_t static radsToTicks(double rads, dxl_motor &motor);
    int32_t static radsPerSecToTicksPerSec(double rads_per_sec, dxl_motor &motor);
    double ticksPerSecToRadsPerSec(int32_t ticks_per_sec, dxl_motor &motor);

};



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
    bool ping (dxl_motor & motor);
    bool setTorque(const dxl_motor &motor, bool flag);
    bool bulkWrite();
    bool bulkRead();
    bool reboot(const dxl_motor &motor);
    bool broadcastPing(std::vector<uint8_t> result_vec);

};


#endif //ARMADILLO_HW_ARM_INTERFACE_H
