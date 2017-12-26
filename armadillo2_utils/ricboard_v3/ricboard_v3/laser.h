#ifndef LASER_H
#define LASER_H

#include <Adafruit_VL53L0X.h>

class Laser
{

private:
	Adafruit_VL53L0X *lox;
  bool init_ok_ = false;

public:

  ~Laser() { delete lox; }
  
	enum Code 
	{
    ERROR = 65535,
    OUT_OF_RANGE = 4
	};
  
	bool init()
  {
    lox = new Adafruit_VL53L0X();
    init_ok_ = lox->begin();
    if (!init_ok_) 
      return false;
    return true;
  }
  
  /*
  * return values:
  * 30 - 8189  real data (mm)
  * 4          object to far
  * 65535      error - reached timeout and read failed 
  */
  uint16_t read()
  {
    if (!init_ok_) 
      return (uint16_t)Code::ERROR;
      
    VL53L0X_RangingMeasurementData_t measure;
    lox->rangingTest(&measure, false); //pass in 'true' to get debug data printout
    return measure.RangeMilliMeter;
  }

};

#endif //LASER_H
