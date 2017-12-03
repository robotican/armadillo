
#include <ric_interface/ric_interface.h>


namespace ric_interface
{
    RicInterface::RicInterface()
    {
        //connect("/dev/RICBOARD");
    }

    void RicInterface::loop()
    {
        keepAliveAndRead();
    }

    void RicInterface::keepAliveAndRead()
    {
        get_keepalive_timer_.startTimer(GET_KA_TIMEOUT);
        if (get_keepalive_timer_.isFinished())
        {
            if (got_keepalive_) //connected
            {
                is_board_alive_ = true;
                got_keepalive_ = false;
                //printf("board alive ! \n");
                get_keepalive_timer_.reset();
            }
            else
            {
                is_board_alive_ = false;
                //printf("board dead ! \n");
                get_keepalive_timer_.reset();
            }
        }

        send_keepalive_timer_.startTimer(SEND_KA_TIMEOUT);
        if (send_keepalive_timer_.isFinished())
        {
            sendKeepAlive();
            send_keepalive_timer_.reset();
        }

        /* try to read header */
        protocol::header incoming_header;
        if (readHeader(incoming_header))
        {
            //printf("INCOMMING header type: %d\n", (int)incoming_header.type);
            handleHeader(incoming_header);
        }
    }

    /* open connection to serial port                */
    /* if conncetion fails, exception will be thrown */
    void RicInterface::connect(std::string port)
    {
        //comm_.connect("/dev/RICBOARD", 115200);
        comm_.connect(port, 115200);
    }


    bool RicInterface::readHeader(protocol::header &h)
    {
        size_t header_size = sizeof(protocol::header);
        byte buff[header_size];
        int bytes_read = comm_.read(buff, header_size);
        if (bytes_read != header_size)
            return false;
        memcpy(&h, buff, header_size);
        //printf("header type: %i, h size: %i, bytes_read: %i\n", h.type, sizeof(h), bytes_read);
        return true;
    }

    void RicInterface::handleHeader(const protocol::header &h)
    {
        switch (h.type)
        {
            case protocol::Type::KEEP_ALIVE:
            {
                got_keepalive_ = true;
                break;
            }
            case protocol::Type::LOGGER:
            {
                protocol::logger logger_pkg;
                if (readLoggerPkg(logger_pkg))
                {
                    printf("logger: %s\n", logger_pkg.msg);
                }
                break;
            }
            case protocol::Type::ULTRASONIC:
            {
                protocol::ultrasonic ultrasonic_pkg;
                if (readUltrasonicPkg(ultrasonic_pkg))
                {
                    sensors_state_.ultrasonic = ultrasonic_pkg;
                    //printf("ultrasonic: %d\n", ultrasonic_pkg.distance_mm);
                }
                break;
            }
            case protocol::Type::IMU:
            {
                protocol::imu imu_pkg;
                if (readImuPkg(imu_pkg))
                {
                    sensors_state_.imu = imu_pkg;
                   /* fprintf(stderr, "imu: roll: %f, pitch: %f, yaw: %f \n", sensors_state_.imu.roll_rad * 180 / M_PI,
                                                                    sensors_state_.imu.pitch_rad * 180 / M_PI,
                                                                    sensors_state_.imu.yaw_rad * 180 / M_PI);*/
                }
                break;
            }
            case protocol::Type::LASER:
            {
                protocol::laser laser_pkg;
                if (readLaserPkg(laser_pkg))
                {
                    sensors_state_.laser = laser_pkg;
                    printf("laser dist: %d\n", sensors_state_.laser.distance_mm);
                }
                break;
            }
        }
    }

    void RicInterface::sendKeepAlive()
    {
        protocol::header keepalive_header;
        keepalive_header.type = protocol::Type::KEEP_ALIVE;
        byte buff[sizeof(protocol::header)];
        memcpy(buff, &keepalive_header, sizeof(protocol::header));
        if (!comm_.send(buff, sizeof(protocol::header)))
            printf("cant send keep alive\n");
    }

    bool RicInterface::readLoggerPkg(protocol::logger &logger_pkg)
    {
        size_t logger_size = sizeof(protocol::logger);
        byte buff[logger_size];
        int bytes_read = comm_.read(buff, logger_size);
        if (bytes_read != logger_size)
            return false;
        memcpy(&logger_pkg, buff, logger_size);
        return true;
    }

    bool RicInterface::readUltrasonicPkg(protocol::ultrasonic &ultrasonic_pkg)
    {
        size_t ultrasonic_size = sizeof(protocol::ultrasonic);
        byte buff[ultrasonic_size];
        int bytes_read = comm_.read(buff, ultrasonic_size);
        if (bytes_read != ultrasonic_size)
            return false;
        memcpy(&ultrasonic_pkg, buff, ultrasonic_size);
        return true;
    }

    bool RicInterface::readImuPkg(protocol::imu &imu_pkg)
    {
        size_t imu_size = sizeof(protocol::imu);
        byte buff[imu_size];
        int bytes_read = comm_.read(buff, imu_size);
        if (bytes_read != imu_size)
            return false;
        memcpy(&imu_pkg, buff, imu_size);
        return true;
    }

    bool RicInterface::readLaserPkg(protocol::laser &laser_pkg)
    {
        size_t laser_size = sizeof(protocol::laser);
        byte buff[laser_size];
        int bytes_read = comm_.read(buff, laser_size);
        if (bytes_read != laser_size)
            return false;
        memcpy(&laser_pkg, buff, laser_size);
        return true;
    }

}