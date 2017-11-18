#include <iostream>
using namespace std;



#include <unistd.h>
#include <board_manager.h>

ric_interface::BoardManager bm;

int main()
{
    /*comm.connect("/dev/cu.usbmodem1411", 115200);
    ric_interface::header h;
    h.type = 10;
    byte buff[sizeof(h)];
    int real_size = comm.readHeader(buff, sizeof(h));
    memcpy(&h, buff, sizeof(h));
    printf("header type: %i, h size: %i, real size: %i", h.type, sizeof(h), real_size);*/

    while (true)
    {
        bm.loop();
        usleep(20);

    }

    return 0;
}




