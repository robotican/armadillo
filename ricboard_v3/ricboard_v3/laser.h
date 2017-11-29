#ifndef LASER_H
#define LASER_H

#include <Wire.h>
#include <VL53L0X.h>


class Laser
{

private:
	VL53L0X sensor_;

public:
	enum Code 
	{
    ERROR = 65535,
    OUT_OF_RANGE = 8190
	};
  
	void init()
  {
    Wire1.begin();
    sensor_.init();
    
    /* cancel libaray inner while loop delay. instead, call */
    /* read() inside loop(), and check for errors           */
    sensor_.setTimeout(0); 
    
    sensor_.setMeasurementTimingBudget(200000);  
  }
  
  /*
  * return values:
  * 30 - 8189  real data (mm)
  * 8190       object to far
  * 65535      error - reached timeout and read failed 
  */
  uint16_t read()
  {
    return sensor_.readRangeSingleMillimeters();   
  }

};

#endif //LASER_H
