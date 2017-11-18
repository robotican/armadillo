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
        const int SEND_KA_TIMEOUT = 300; //ms
        const int GET_KA_TIMEOUT = 1000; //ms

        /* is board sent keep alive on time */
        bool is_board_alive_, got_keepalive_;
        Timer send_keepalive_timer_, get_keepalive_timer_;
        ric_interface::Communicator comm_;

        void connect();
        bool readHeader(protocol::header &h);
        void sendKeepAlive();
        void handleHeader(const protocol::header &h);

    public:
        BoardManager();
        void loop();
        bool isBoardAlive() { return is_board_alive_; }

    };
}



#endif //RIC_INTERFACE_BOARD_MANAGER_H
