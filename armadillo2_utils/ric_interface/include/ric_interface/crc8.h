/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elchay Rauper*/


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


    byte get_byte_crc(byte val)
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

    /* build crc table */
    void init()
    {
        int i;
        // fill an array with CRC values of all 256 possible bytes
        for (i = 0; i < 256; i++)
            crc_table_[i] = get_byte_crc(i);
    }

public:

    byte get_crc(byte message[], size_t size)
    {
        init();
        byte i, crc = 0;
        for (i = 0; i < size; i++)
            crc = crc_table_[crc ^ message[i]];
        return crc;
    }
};

#endif //RIC_INTERFACE_CRC8_H
