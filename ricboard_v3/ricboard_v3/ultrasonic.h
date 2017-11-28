#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

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
  }

  /* returns distance in mm */
  uint16_t readDistanceMm()
  {
    /* each micro second of pulse lenght is 1 mm */
    return (uint16_t)pulseIn(pin_, HIGH);
  }
};
#endif //ULTRASONIC_H

