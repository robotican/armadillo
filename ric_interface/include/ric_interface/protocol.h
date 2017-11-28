#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

namespace ric_interface
{
    namespace protocol
    {
        enum class Type
        {
            KEEP_ALIVE = 1,
            LOGGER = 2,
            ULTRASONIC = 3,
            LIDAR = 4,
            IMU = 5
        };

        struct header
        {
            Type type;
        };

        struct package
        {

        };

        struct logger : package
        {
            enum Code
            {
                INFO = 1,
                WARNING = 2,
                ERROR = 3
            };
            char msg[128];
            Code code;
        };

        struct sensor : package
        {

        };

        struct actuator : package
        {
        };

        struct ultrasonic : sensor
        {
            uint16_t distance_mm;
        };

        struct lidar : sensor
        {
            uint16_t distance_mm;
        };

        struct imu : sensor
        {
            float roll,
                    pitch,
                    yaw,
                    accl_x,
                    accl_y,
                    accl_z,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    mag_x,
                    mag_y,
                    mag_z;
        };

        struct elevator : actuator
        {
            uint16_t cmd_mm;
        };
    }
}



#endif //PROTOCOL_H