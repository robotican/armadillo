#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include "timer.h"
#define USONIC_SCALE_FACT 5

class Ultrasonic
{
private:
 int pin_;
 Timer timer;

public:
  void init(int pin)
  {
    pin_ = pin;
    pinMode(pin_, INPUT);
    analogReadResolution(10);
    /* sensor rate is 10Hz according to data sheet */
    timer.start(100);
  }

  /* returns distance in mm */
  bool readDistanceMm(uint16_t &value)
  {
    if (timer.finished())
    {
      value = (uint16_t)analogRead(pin_) * USONIC_SCALE_FACT;
      return true;
    }
    return false;
  }
};
#endif //ULTRASONIC_H

