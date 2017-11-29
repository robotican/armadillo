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
            LASER = 4,
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

        struct laser : sensor
        {
            uint16_t distance_mm;
        };

        struct imu : sensor
        {
            float roll_rad,
                    pitch_rad,
                    yaw_rad,
                    accl_x_rad,
                    accl_y_rad,
                    accl_z_rad,
                    gyro_x_rad,
                    gyro_y_rad,
                    gyro_z_rad,
                    mag_x_rad,
                    mag_y_rad,
                    mag_z_rad;
        };

        struct elevator : actuator
        {
            uint16_t cmd_mm;
        };
    }
}



#endif //PROTOCOL_H