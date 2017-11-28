
#include <ric_interface/board_manager.h>

namespace ric_interface
{
    BoardManager::BoardManager()
    {
        connect();
    }

    void BoardManager::loop()
    {
        keepAliveAndRead();
    }

    void BoardManager::keepAliveAndRead()
    {
        get_keepalive_timer_.startTimer(GET_KA_TIMEOUT);
        if (get_keepalive_timer_.isFinished())
        {
            if (got_keepalive_) //connected
            {
                is_board_alive_ = true;
                got_keepalive_ = false;
                printf("board alive ! \n");
                get_keepalive_timer_.reset();
            }
            else
            {
                is_board_alive_ = false;
                printf("board dead ! \n");
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
            printf("INCOMMING header type: %d\n", (int)incoming_header.type);
            handleHeader(incoming_header);
        }
    }

    /* open connection to serial port                */
    /* if conncetion fails, exception will be thrown */
    void BoardManager::connect()
    {
        comm_.connect("/dev/RICBOARD", 115200); //TODO: GET FROM PARAM YAML FILE
    }


    bool BoardManager::readHeader(protocol::header &h)
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

    void BoardManager::handleHeader(const protocol::header &h)
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
        }
    }

    void BoardManager::sendKeepAlive()
    {
        protocol::header keepalive_header;
        keepalive_header.type = protocol::Type::KEEP_ALIVE;
        byte buff[sizeof(protocol::header)];
        memcpy(buff, &keepalive_header, sizeof(protocol::header));
        if (!comm_.send(buff, sizeof(protocol::header)))
            printf("cant send keep alive\n");
    }

    bool BoardManager::readLoggerPkg(protocol::logger &logger_pkg)
    {
        size_t logger_size = sizeof(protocol::logger);
        byte buff[logger_size];
        int bytes_read = comm_.read(buff, logger_size);
        if (bytes_read != logger_size)
            return false;
        memcpy(&logger_pkg, buff, logger_size);
        return true;
    }

    bool BoardManager::readUltrasonicPkg(protocol::ultrasonic &ultrasonic_pkg)
    {
        size_t ultrasonic_size = sizeof(protocol::ultrasonic);
        byte buff[ultrasonic_size];
        int bytes_read = comm_.read(buff, ultrasonic_size);
        if (bytes_read != ultrasonic_size)
            return false;
        memcpy(&ultrasonic_pkg, buff, ultrasonic_size);
        return true;
    }
}