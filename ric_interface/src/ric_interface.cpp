
#include <ric_interface/ric_interface.h>


namespace ric_interface
{
    RicInterface::RicInterface()
    {
        connect("/dev/armadillo2/RICBOARD");
    }

    /* open connection to serial port                */
    /* if conncetion fails, exception will be thrown */
    void RicInterface::connect(std::string port)
    {
        comm_.connect(port, 115200);
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
                puts("RIC DEAD");
                is_board_alive_ = false;
                //printf("board dead ! \n");
                get_keepalive_timer_.reset();
            }
        }

        send_keepalive_timer_.startTimer(SEND_KA_TIMEOUT);
        if (send_keepalive_timer_.isFinished())
        {
            protocol::header ka_header;
            ka_header.type = protocol::Type::KEEP_ALIVE;
            protocol::keepalive ka_pkg;
            sendPkg(ka_header, ka_pkg, sizeof(ka_pkg));
            send_keepalive_timer_.reset();
        }

        /* try to read header */
        protocol::header incoming_header;
        if (readPkg(incoming_header, sizeof(incoming_header)))
        {
            //printf("INCOMMING header type: %d\n", (int)incoming_header.type);
            handleHeader(incoming_header);
        }
    }

    void RicInterface::handleHeader(const protocol::header &h)
    {
        switch (h.type)
        {
            case protocol::Type::KEEP_ALIVE:
            {
                got_keepalive_ = true;
                protocol::keepalive ka_pkg;
                if (readPkg(ka_pkg, sizeof(ka_pkg)))
                {
                    //ka pkg is empty
                }
                break;
            }
            case protocol::Type::LOGGER:
            {
                protocol::logger logger_pkg;
                if (readPkg(logger_pkg, sizeof(logger_pkg)))
                {
                    //printf("logger: %s\n", logger_pkg.msg);
                }
                break;
            }
            case protocol::Type::ULTRASONIC:
            {
                protocol::ultrasonic ultrasonic_pkg;
                if (readPkg(ultrasonic_pkg, sizeof(ultrasonic_pkg)))
                {
                    sensors_state_.ultrasonic = ultrasonic_pkg;
                    //printf("ultrasonic: %d\n", ultrasonic_pkg.distance_mm);
                }
                break;
            }
            case protocol::Type::IMU:
            {
                protocol::imu imu_pkg;
                if (readPkg(imu_pkg, sizeof(imu_pkg)))
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
                if (readPkg(laser_pkg, sizeof(laser_pkg)))
                {
                    sensors_state_.laser = laser_pkg;
                    //printf("laser dist: %d\n", sensors_state_.laser.distance_mm);
                }
                break;
            }
            case protocol::Type::GPS:
            {
                protocol::gps gps_pkg;
                if (readPkg(gps_pkg, sizeof(gps_pkg)))
                {
                    sensors_state_.gps = gps_pkg;
                    //printf("gps lat: %f, lon: %f\n", sensors_state_.gps.lat, sensors_state_.gps.lon);
                }
                break;
            }
        }
    }

    bool RicInterface::readPkg(protocol::package &pkg, size_t pkg_size)
    {
        byte buff[pkg_size];
        int bytes_read = comm_.read(buff, pkg_size);
        if (bytes_read != pkg_size)
            return false;
        memcpy(&pkg, buff, pkg_size);
        //printf("header type: %i, h size: %i, bytes_read: %i\n", h.type, sizeof(h), bytes_read);
        return true;
    }

    /* send header and then state (i.e. keep alive) pkg content to ricboard */
    bool RicInterface::sendPkg(const protocol::header &header_pkg,
                               const protocol::package &pkg,
                               size_t pkg_size)
    {
        /* send header */
        size_t header_size = sizeof(protocol::header);
        byte header_buff[header_size];
        memcpy(header_buff, &header_pkg, header_size);
        if (!comm_.send(header_buff, header_size))
            return false;

        /* send pkg */
        byte pkg_buff[pkg_size];
        memcpy(pkg_buff, &pkg, pkg_size);
        if (!comm_.send(pkg_buff, pkg_size))
            return false;
        return true;
    }

    void RicInterface::writeCmd(const protocol::actuator &actu_pkg, size_t size, protocol::Type type)
    {
        protocol::header header_pkg;
        header_pkg.type = type;
        sendPkg(header_pkg, actu_pkg, size);
    }
}