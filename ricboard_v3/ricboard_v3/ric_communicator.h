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
        void sendPkg(const protocol::package &pkg, 
                     size_t pkg_size)
        {
          /* send pkg */
          byte pkg_buff[pkg_size];
          memcpy(pkg_buff, &pkg, pkg_size);
          send(pkg_buff, pkg_size);
        }

        bool sendHeaderAndPkg(protocol::Type header_type,
                              const protocol::package &pkg,
                              size_t pkg_size)
        {
            protocol::header header_pkg;
            header_pkg.type = header_type;
            /* send header */
            sendPkg(header_pkg, sizeof(protocol::header));
            /* send pkg with content */
            sendPkg(pkg, pkg_size);
        }
    }
}
#endif //RIC_COMMUNICATOR_H
