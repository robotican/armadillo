
#include <ric_interface/crc8.h>

byte Crc8::getByteCrc(byte val)
{
    byte j;

    for (j = 0; j < 8; j++)
    {
        if (val & 1)
            val ^= CRC7_POLY_;
        val >>= 1;
    }
    return val;
}

void Crc8::init()
{
    int i;
    // fill an array with CRC values of all 256 possible bytes
    for (i = 0; i < 256; i++)
        crc_table_[i] = getByteCrc(i);
}

byte Crc8::getCrc(byte *message, size_t size)
{
    init();
    byte i, crc = 0;
    for (i = 0; i < size; i++)
        crc = crc_table_[crc ^ message[i]];
    return crc;
}





