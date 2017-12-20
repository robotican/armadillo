#ifndef PROTOCOL_H
#define PROTOCOL_H

namespace protocol
{
    enum class Type
    {
        KEEP_ALIVE = 100,
        LOGGER = 101,
        ULTRASONIC = 102,
        LASER = 103,
        IMU = 104,
        GPS = 105,
        SERVO = 106
    };

    const uint16_t MAX_PKG_SIZE = 512;
    const uint8_t HEADER_CODE = 200;
    const uint8_t HEADER_INDX = 0;
    const uint8_t PKG_SIZE_INDX = 1;

    struct package
    {
        uint8_t type;
        uint8_t checksum;
    };

    struct keepalive : package
    {
        keepalive() { type = (uint8_t)Type::KEEP_ALIVE; }
    };

    struct logger : package
    {
        logger() { type = (uint8_t)Type::LOGGER; }
        char msg[30];
        int32_t value;
    };

    struct sensor : package
    {

    };

    struct actuator : package
    {
    };

    struct ultrasonic : sensor
    {
        ultrasonic() { type = (uint8_t)Type::ULTRASONIC; }
        uint16_t distance_mm;
    };

    struct laser : sensor
    {
        laser() { type = (uint8_t)Type::LASER; }
        uint16_t distance_mm;
    };

    struct imu : sensor
    {
        imu() { type = (uint8_t)Type::IMU; }
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

    struct gps : sensor
    {
        gps() { type = (uint8_t)Type::GPS; }
        float lat,
                lon;

        float speed,
                angle;

        float elevation;

        /* E, N, S, W */
        char lat_mark,
                lon_mark;

        uint8_t hour,
                minute,
                seconds,
                year,
                month,
                day;

        uint8_t fix_quality,
                satellites;

        bool fix;
    };


    struct servo : actuator
    {
        servo() { type = (uint8_t)Type::SERVO; }
        uint16_t cmd; //servo command 1000-2000
    };
}




#endif //PROTOCOL_H
