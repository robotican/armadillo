#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#define USONIC_SCALE_FACT 5

class Ultrasonic
{
private:
 int pin_;
 long pulse_length_;

public:
  void init(int pin)
  {
    pin_ = pin;
    pinMode(pin_, INPUT);
    analogReadResolution(10);
  }

  /* returns distance in mm */
  uint16_t readDistanceMm()
  {
    return (uint16_t)analogRead(pin_) * USONIC_SCALE_FACT;
  }
};
#endif //ULTRASONIC_H

