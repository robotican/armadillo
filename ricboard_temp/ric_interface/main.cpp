#include <iostream>
using namespace std;



#include <unistd.h>
#include <board_manager.h>

ric_interface::BoardManager bm;

int main()
{

    while (true)
    {
        bm.loop();
        usleep(20);

    }

    return 0;
}




