#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

namespace ric_interface
{
    namespace protocol
    {
        enum class Type
        {
            KEEP_ALIVE = 1,
            LOGGER = 2,
            ULTRASONIC = 3,

        };

        struct header
        {
            Type type;
        };

        struct package
        {

        };

        struct logger : package
        {
            char msg[128];
            uint8_t code;
        };

        struct sensor : package
        {

        };

        struct ultrasonic : sensor
        {
            uint16_t distance_mm;
        };
    }
}



#endif //PROTOCOL_H