#ifndef PULLUP_PIN_H
#define PULLUP_PIN_H

#include <Arduino.h>

class PullupPin
{
private:
 int pin_;

public:
  void init(int pin)
  {
    pin_ = pin;
    pinMode(pin_, INPUT_PULLUP);
  }
  /* return false for 0 or true for 1 */
  bool readState()
  {
    return (bool)digitalRead(pin_);
  }
};
#endif //PULLUP_PIN_H

