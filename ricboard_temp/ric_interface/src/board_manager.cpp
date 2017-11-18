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
        keepalive_timer_.startTimer(KEEP_ALIVE_TIMEOUT);
        if (keepalive_timer_.isFinished())
        {
            if (got_keepalive_)
            {
                is_board_alive_ = true;
                got_keepalive_ = false;
                printf("board alive ! \n");
            }
            else
            {
                is_board_alive_ = false;
                printf("board dead ! \n");
            }
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
        h.type = 10;
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
}