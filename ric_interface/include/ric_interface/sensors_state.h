//
// Created by armadillo2 on 27/11/17.
//

#ifndef RIC_INTERFACE_SENSORSSTATE_H
#define RIC_INTERFACE_SENSORSSTATE_H

#include "protocol.h"

namespace ric_interface
{
    struct sensors_state
    {
        protocol::ultrasonic ultrasonic;
        protocol::imu imu;
        protocol::laser laser;
    };
}


#endif //RIC_INTERFACE_SENSORSSTATE_H
