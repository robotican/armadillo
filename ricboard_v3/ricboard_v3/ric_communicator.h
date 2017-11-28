#ifndef RIC_COMMUNICATOR_H
#define RIC_COMMUNICATOR_H

#include <Arduino.h>
#include "communicator.h"
#include "protocol.h"

namespace communicator
{
    namespace ric
    {
        
        bool readHeader(protocol::header &header_pkg)
        {
            byte buff[sizeof(protocol::header)];
            int bytes_read = read(buff, sizeof(protocol::header));
            if (bytes_read != sizeof(protocol::header))
                return false;
            memcpy(&header_pkg, buff, sizeof(protocol::header));
            return true;
        }
        
        void sendHeader(const protocol::header &header_pkg)
        {
            byte buff[sizeof(protocol::header)];
            memcpy(buff, &header_pkg, sizeof(protocol::header));
            send(buff, sizeof(protocol::header));
        }
        
        void sendKeepAlive()
        {
            protocol::header keepalive_header;
            keepalive_header.type = protocol::Type::KEEP_ALIVE;
            sendHeader(keepalive_header);
        }

        void sendUltrasonic(const protocol::ultrasonic &ultrasonic_pkg)
        {
            protocol::header ultrasonic_header;
            ultrasonic_header.type = protocol::Type::ULTRASONIC;
            sendHeader(ultrasonic_header);

            size_t ultasonic_size = sizeof(protocol::ultrasonic);
            byte buff[ultasonic_size];
            memcpy(buff, &ultrasonic_pkg, ultasonic_size);
            send(buff, ultasonic_size);
        }

        void sendLogger(const protocol::logger &logger_pkg) 
        {
            protocol::header logger_header;
            logger_header.type = protocol::Type::LOGGER;
            sendHeader(logger_header);

            size_t logger_size = sizeof(protocol::logger);
            byte buff[logger_size];
            memcpy(buff, &logger_pkg, logger_size);
            send(buff, logger_size);
        }
    }
}
#endif //RIC_COMMUNICATOR_H
