#ifndef RIC_COMMUNICATOR_H
#define RIC_COMMUNICATOR_H

#include <Arduino.h>
#include "communicator.h"
#include "protocol.h"

namespace communicator
{
    namespace ric
    {
        bool readPkg(protocol::package &pkg, size_t pkg_size)
        {
          byte buff[pkg_size];
          int bytes_read = read(buff, pkg_size);
          if (bytes_read != pkg_size)
              return false;
          memcpy(&pkg, buff, pkg_size);
          return true;
        }
        
        /* send header and then sensor state pkg content to pc */
        void sendPkg(const protocol::header &header_pkg,
                     const protocol::package &pkg, 
                     size_t pkg_size)
        {
          /* send header */
          size_t header_size = sizeof(protocol::header);
          byte header_buff[header_size];
          memcpy(header_buff, &header_pkg, header_size);
          send(header_buff, header_size);

          /* send pkg */
          byte pkg_buff[pkg_size];
          memcpy(pkg_buff, &pkg, pkg_size);
          send(pkg_buff, pkg_size);
        }
    }
}
#endif //RIC_COMMUNICATOR_H
