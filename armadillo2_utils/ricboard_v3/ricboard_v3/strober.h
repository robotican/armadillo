/****************************************************************************
* Copyright (c) 2017, BlueSky Software
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. All advertising materials mentioning features or use of this software
*    must display the following acknowledgement:
*    This product includes software developed by  BlueSky Software.
* 4. Neither the name of the BlueSky Software nor the
*    names of its contributors may be used to endorse or promote products
*    derived from this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY BlueSky Software ''AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL BlueSky Software BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

#ifndef LED_H
#define LED_H

#include "timer.h"

class Strober
{

public:
  enum Notes 
  {
    BLINK_FAST,
    BLINK_SLOW,
    STROBE
  };

  /**************************************************************
  * Goal: Play notes (blink led accordingly). This method must
  * be invoked in loop for continuouse play.
  * Param: pin of led to blink
  **************************************************************/
  void play(uint16_t pin)
  {
    if (index_ < notes_size_)
    {
      if (timer_.finished())
      {
        if (index_ % 2 == 0)
        {
          if (notes_[index_] == 0)
            digitalWrite(pin, LOW);
          else
            digitalWrite(pin, HIGH);
        }
        else
          timer_.start(notes_[index_]);
        index_++;
      }
    }
    else
      index_ = 0;
  }
  void setNotes(uint16_t* notes, size_t notes_size)
  {
    if (notes_ != notes)
    {
      notes_ = notes;
      notes_size_ = notes_size;
      timer_.reset();
    }
  }
  void setNotes(Notes notes)
  {
    if (curr_notes_ != notes)
    {
      curr_notes_ = notes;
      timer_.reset();
      switch (notes)
      {
        case BLINK_SLOW:
          notes_ = (uint16_t*)SLOW_BLINK_ARR;
          notes_size_ = SLOW_BLINK_SIZE;
          break;
        case BLINK_FAST:
          notes_ = (uint16_t*)FAST_BLINK_ARR;
          notes_size_ = FAST_BLINK_SIZE;
          break;
        case STROBE:
          notes_ = (uint16_t*)STROBE_BLINK_ARR;
          notes_size_ = STROBE_BLINK_SIZE;
          break;
      }
    }
  }

  const static size_t SLOW_BLINK_SIZE = 4;
  const static size_t FAST_BLINK_SIZE = 4;
  const static size_t STROBE_BLINK_SIZE = 12;

private:
  Timer timer_;
  int16_t pin_;
  uint16_t* notes_;
  size_t notes_size_;
  uint16_t index_ = 0;
  Notes curr_notes_;

  const uint16_t SLOW_BLINK_ARR[SLOW_BLINK_SIZE] = {1, 500, 0, 500};
  const uint16_t FAST_BLINK_ARR[FAST_BLINK_SIZE] = {1, 300, 0, 300};
  const uint16_t STROBE_BLINK_ARR[STROBE_BLINK_SIZE] = {1, 70, 0, 70, 1, 70, 0, 70, 1, 70, 0, 500};

  
};
#endif
