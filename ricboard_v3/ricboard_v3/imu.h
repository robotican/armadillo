#ifndef IMU_H
#define IMU_H

#include <Wire.h>
//#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <RTIMULib.h>
#include "protocol.h"

class Imu
{
  private:
    RTIMU *imu_;                                          
    RTIMUSettings *settings_;      
  
  public: 
    
    bool init();
    bool read(protocol::imu &data);
};
#endif //IMU_H
