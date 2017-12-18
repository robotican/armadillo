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

    struct package
    {

    };

    struct header : package
    {
        Type type;
    };

    struct keepalive : package
    {

    };

    struct logger : package
    {
        char msg[128];
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

    struct gps : sensor
    {
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
        uint16_t cmd; //servo command 1000-2000
    };
}



#endif //PROTOCOL_H
