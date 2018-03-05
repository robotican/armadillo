

#ifndef RIC_INTERFACE_COMMUNICATOR_H
#define RIC_INTERFACE_COMMUNICATOR_H

#include <ric_interface/protocol.h>
#include <ric_interface/serial_com.h>
#include <string.h>
#include <iostream>
#include "crc8.h"



namespace ric
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

        void connect(std::string port, int baudrate);

        /* return -1 for reading in process, or type of the incoming pkg */
        int read(byte buff[]);

        static void fromBytes(byte buff[], size_t pkg_size, protocol::package &pkg);

        static void toBytes(const protocol::package &pkg, size_t pkg_size, byte buff[]);

        bool write(const protocol::package &pkg, size_t pkg_size);

    private:
        SerialCom serial_;
        State state_ = HEADER_PART_A;
        int pkg_indx_ = 0;
        int pkg_size_ = 0;
        Crc8 crc_;

        void reset();

        /* try to read valid header start */
        bool tryReadHeader();

        /* return -1 for failure and pkg size as success */
        int tryReadPkgSize();
    };
}

#endif //RIC_INTERFACE_COMMUNICATOR_H
