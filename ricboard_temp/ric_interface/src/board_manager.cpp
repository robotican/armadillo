//
// Created by Eli Eli on 18/11/2017.
//

#include "board_manager.h"

namespace ric_interface
{
    BoardManager::BoardManager()
    {
        connect();
    }

    void BoardManager::loop()
    {
        get_keepalive_timer_.startTimer(GET_KA_TIMEOUT);
        if (get_keepalive_timer_.isFinished())
        {
            if (got_keepalive_)
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
            printf("INCOMMING header type: %i\n", incoming_header.type);
            handleHeader(incoming_header);
        }

        //TODO: send ka to board
    }

    /* open connection to serial port                */
    /* if conncetion fails, exception will be thrown */
    void BoardManager::connect()
    {
        comm_.connect("/dev/cu.usbmodem1411", 115200); //TODO: GET FROM PARAM YAML FILE
    }


    bool BoardManager::readHeader(protocol::header &h)
    {
        byte buff[sizeof(h)];
        int bytes_read = comm_.read(buff, sizeof(h));
        if (bytes_read != sizeof(h))
            return false;
        memcpy(&h, buff, sizeof(h));
        //printf("header type: %i, h size: %i, bytes_read: %i\n", h.type, sizeof(h), bytes_read);
        return true;
    }

    void BoardManager::handleHeader(const protocol::header &h)
    {
        switch (h.type)
        {
            case (int)protocol::Type::KEEP_ALIVE:
                got_keepalive_ = true;
                break;
        }
    }

    void BoardManager::sendKeepAlive()
    {
        protocol::header keepalive_header;
        keepalive_header.type = (uint8_t)protocol::Type::KEEP_ALIVE;//(uint8_t)protocol::Type::KEEP_ALIVE;
        byte buff[sizeof(keepalive_header)];
        memcpy(buff, &keepalive_header, sizeof(keepalive_header));
        if (!comm_.send(buff, sizeof(keepalive_header)))
            printf("cant send keep alive\n");
    }
}