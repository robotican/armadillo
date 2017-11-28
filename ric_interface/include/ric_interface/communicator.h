#ifndef RIC_INTERFACE_COMMUNICATOR_H
#define RIC_INTERFACE_COMMUNICATOR_H



#include "ric_exception.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

typedef unsigned char byte;

#include <string>

namespace ric_interface
{
    class Communicator
    {

    private:
        int file_handle_,
                baudrate_;
        void setAttributes();


    public:
        void connect(std::string port, int baudrate);
        bool send(const byte buff[], size_t size);
        int read(byte buff[], size_t size);

    };
}



#endif //RIC_INTERFACE_COMMUNICATOR_H
