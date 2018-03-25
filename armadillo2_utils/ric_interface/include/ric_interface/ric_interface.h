/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


#ifndef RIC_INTERFACE_BOARD_MANAGER_H
#define RIC_INTERFACE_BOARD_MANAGER_H

#include "communicator.h"
#include "timer.h"
#include "protocol.h"
#include "sensors_state.h"
#include <math.h>
#include <string.h>
#include <iostream>

namespace ric_interface
{
    class RicInterface
    {
    private:
        const int SEND_KA_TIMEOUT = 300; //ms
        const int GET_KA_TIMEOUT = 1500; //ms

        /* is board sent keep alive on time */
        bool is_board_alive_ = true, got_keepalive_ = false;
        bool got_new_logger_msg_ = false, got_new_error_msg_ = false;
        Timer send_keepalive_timer_, get_keepalive_timer_;
        Communicator comm_;
        sensors_state sensors_state_;
        byte pkg_buff_[protocol::MAX_PKG_SIZE];

        void readAndHandlePkg();
        void checkKeepAliveFromRic();
        void sendKeepAlive();
        void clearBuffer();

    public:
        RicInterface();
        void connect(std::string port);
        void loop();
        bool isBoardAlive() { return is_board_alive_; }
        sensors_state getSensorsState() { return sensors_state_; }
        void writeCmd(const protocol::actuator &actu_pkg, size_t size, protocol::Type type);
        bool readLoggerMsg(std::string &msg, int32_t &value);
        bool readErrorMsg(protocol::error &error);
        static std::string compType2String(const protocol::Type comp_type);
        static std::string errCode2String(const protocol::ErrCode err_code);
    };
}



#endif //RIC_INTERFACE_BOARD_MANAGER_H
