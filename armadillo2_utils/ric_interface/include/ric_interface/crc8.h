
#ifndef RIC_INTERFACE_CRC8_H
#define RIC_INTERFACE_CRC8_H

#include <stdio.h>
#include <iostream>

typedef uint8_t byte;

class Crc8
{

private:
    const byte CRC7_POLY_ = 0x91;
    byte crc_table_[256];


    byte getByteCrc(byte val);

    /* build crc table */
    void init();

public:

    byte getCrc(byte *message, size_t size);
};

#endif //RIC_INTERFACE_CRC8_H
