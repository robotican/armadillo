#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <Arduino.h>

namespace communicator
{
    void init(int baudrate)
    {
      Serial.begin(baudrate);
    }

    void send(byte buff[], size_t size)
    {
      Serial.write(buff, size);
    }

    /* return: number of bytes read */
    int read(byte buff[], size_t size)
    {
      bool no_bytes = true;
      int indx=0;
              while (indx<size && Serial.available()>0)
              {
          no_bytes = false;
          int incoming_byte = Serial.read();
          if (incoming_byte != -1) 
                      buff[indx++] = (byte)incoming_byte;
              };
      if (no_bytes)
          return -1;
              return indx;
    }
}

#endif //COMMUNICATOR_H
