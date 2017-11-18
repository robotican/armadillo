//
// Created by Eli Eli on 18/11/2017.
//

#ifndef RIC_INTERFACE_BOARD_MANAGER_H
#define RIC_INTERFACE_BOARD_MANAGER_H

#include "communicator.h"
#include "timer.h"
#include "protocol.h"

namespace ric_interface
{
    class BoardManager
    {
    private:
        const int KEEP_ALIVE_TIMEOUT = 500; //ms

        /* is board sent keep alive on time */
        bool is_board_alive_, got_keepalive_;
        Timer keepalive_timer_;
        ric_interface::Communicator comm_;

        void connect();
        bool readHeader(protocol::header &h);
        void handleHeader(const protocol::header &h);

    public:
        BoardManager();
        void loop();
        bool isBoardAlive() { return is_board_alive_; }

    };
}



#endif //RIC_INTERFACE_BOARD_MANAGER_H
