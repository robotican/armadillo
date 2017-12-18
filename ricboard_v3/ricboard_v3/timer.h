#ifndef TIMER_H
#define TIMER_H
#include <Arduino.h>

class Timer
{
private:
    unsigned long start_time_;
    unsigned long end_time_;
    unsigned long period_;
    bool started_ = false;

public:
    void start(unsigned long period)
    {
      if (!started_)
      {
        period_ = period;
        start_time_ = millis();
        started_ = true;
      }
    }
    
    void startOver()
    {
      start_time_ = millis();
      if (!started_)
        started_ = true;
    }
    
    bool finished()
    {
      if (started_)
      {
          end_time_ = millis();
          if (end_time_ - start_time_ >= period_)
          {
              started_ = false;
          }
      }
      return !started_;
    }
    
    void reset() { started_ = false; }
};

#endif
