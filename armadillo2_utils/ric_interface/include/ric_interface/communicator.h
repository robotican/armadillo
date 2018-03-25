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
/* Author: Elchay Rauper*/


#ifndef RIC_INTERFACE_COMMUNICATOR_H
#define RIC_INTERFACE_COMMUNICATOR_H

#include <ric_interface/protocol.h>
#include <ric_interface/serial_com.h>
#include <string.h>
#include <iostream>
#include "crc8.h"



namespace ric_interface
{
    typedef uint8_t byte;
    class Communicator
    {

    public:

        enum State
        {
            HEADER_PART_A,
            HEADER_PART_B,
            PACKAGE,
            CHECKSUM
        };

        void connect(std::string port, int baudrate)
        {
            serial_.connect(port, baudrate);
        }

        /* return -1 for reading in process, or type of the incoming pkg */
        int read(byte buff[])
        {
            switch (state_)
            {
                case HEADER_PART_A: //read header
                {
                    if (tryReadHeader())
                        state_ = HEADER_PART_B;
                    break;
                }
                case HEADER_PART_B: //read pkg size
                {
                    pkg_size_ = tryReadPkgSize();
                    if (pkg_size_ != -1)
                        state_ = PACKAGE;
                    break;
                }
                case PACKAGE:
                {
                    int incoming = serial_.read();
                    if (incoming != -1)
                        buff[pkg_indx_++] = (byte)incoming;

                    if (pkg_indx_ >= pkg_size_) //done reading pkg content
                        state_ = CHECKSUM;
                    break;
                }
                case CHECKSUM:
                {

                    int incoming = serial_.read();
                    if (incoming != -1)
                    {
                        byte incoming_checksum = (byte)incoming;
                        byte computed_checksum = crc_.get_crc(buff, pkg_size_);

                        //fprintf(stderr, "got chksum: %d, comp chksum: %d\n", incoming_checksum, computed_checksum);

                        if (incoming_checksum == computed_checksum)
                        {
                            protocol::package pkg;
                            fromBytes(buff, sizeof(protocol::package), pkg);
                            reset();
                            return (uint8_t)pkg.type;
                        }
                        else
                            return -2; //wrong checksum
                    }
                    break;
                }
            }
            return -1;
        }

        static void fromBytes(byte buff[], size_t pkg_size, protocol::package &pkg)
        {
            memcpy(&pkg, buff, pkg_size);
        }

        static void toBytes(const protocol::package &pkg, size_t pkg_size, byte buff[])
        {
            memcpy(buff, &pkg, pkg_size);
        }

        bool write(const protocol::package &pkg, size_t pkg_size)
        {
            /* send pkg header */
            byte header_buff[2];
            header_buff[protocol::HEADER_INDX] = protocol::HEADER_CODE;
            header_buff[protocol::PKG_SIZE_INDX] = pkg_size;
            if (!serial_.send(header_buff, 2))
                return false;
            byte pkg_buff[pkg_size];
            toBytes(pkg, pkg_size, pkg_buff);
            byte checksum[1];
            checksum[0] = crc_.get_crc(pkg_buff, pkg_size);
            /* send pkg content and the checksum */
            if (!serial_.send(pkg_buff, pkg_size))
                return false;
            if (!serial_.send(checksum, 1))
                return false;
            return true;
        }


    private:
        SerialCom serial_;
        State state_ = HEADER_PART_A;
        int pkg_indx_ = 0;
        int pkg_size_ = 0;
        Crc8 crc_;

        void reset()
        {
            state_ = HEADER_PART_A;
            pkg_indx_ = 0;
            pkg_size_ = 0;
        }

        /* try to read valid header start */
        bool tryReadHeader()
        {
            if (serial_.read() == protocol::HEADER_CODE)
                return true;
            return false;
        }

        /* return -1 for failure and pkg size as success */
        int tryReadPkgSize()
        {
            return serial_.read();
        }
    };
}

#endif //RIC_INTERFACE_COMMUNICATOR_H
