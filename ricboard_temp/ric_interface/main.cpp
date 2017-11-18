#include <iostream>
using namespace std;

#include "include/communicator.h"
#include "include/protocol.h"

ric_interface::Communicator comm;

int main()
{
    comm.connect("/dev/cu.usbmodem1411", 115200);
    protocol::header h;
    h.type = 10;
    byte buff[sizeof(h)];
    int real_size = comm.read(buff, sizeof(h));
    memcpy(&h, buff, sizeof(h));
    printf("header type: %i, h size: %i, real size: %i", h.type, sizeof(h), real_size);
    return 0;
}


#include <termios.h>

#include <fcntl.h>
#include <unistd.h>  // UNIX standard function definitions

//#include "ric_exception.h"



