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
    
    bool init()
    {
        int errcode;
        Wire.begin();
        settings_ = new RTIMUSettings();
        imu_ = RTIMU::createIMU(settings_);
    
        Serial.print("TeensyIMU starting using device "); Serial.println(imu_->IMUName());
        if ((errcode = imu_->IMUInit()) < 0) 
           Serial.print("Failed to init IMU: "); Serial.println(errcode); //return false; /* Failed to init IMU */
        
        if (imu_->getCompassCalibrationValid())
          Serial.println("Using compass calibration");
        else
          Serial.println("No valid compass calibration data found");  
      
        imu_->setSlerpPower(0.02);
        imu_->setGyroEnable(true);
        imu_->setAccelEnable(true);
        imu_->setCompassEnable(true);
        return true;
    }

    /* read IMU if available. IMU read must be called */
    /* as fast as possible (no delays*                */
    bool read(protocol::imu &data)
    {
      /* get the latest data if ready yet */
      if (imu_->IMURead()) 
      {                                
        RTIMU_DATA imu_data_raw = imu_->getIMUData();
        data.roll_rad = imu_data_raw.fusionPose.x();
        data.pitch_rad = imu_data_raw.fusionPose.y();
        data.yaw_rad = imu_data_raw.fusionPose.z();
        data.accl_x_rad = imu_data_raw.accel.x();
        data.accl_y_rad = imu_data_raw.accel.y();
        data.accl_z_rad = imu_data_raw.accel.z();
        data.gyro_x_rad = imu_data_raw.gyro.x();
        data.gyro_y_rad = imu_data_raw.gyro.y();
        data.gyro_z_rad = imu_data_raw.gyro.z();
        data.mag_x_rad = imu_data_raw.compass.x();
        data.mag_y_rad = imu_data_raw.compass.y();
        data.mag_z_rad = imu_data_raw.compass.z();
      
        return true;
      }
      return false;
    }

};
#endif //IMU_H
