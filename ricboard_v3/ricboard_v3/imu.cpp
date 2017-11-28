#include "imu.h"


bool Imu::init()
{
    int errcode;
    Wire.begin();
    settings_ = new RTIMUSettings();
    imu_ = RTIMU::createIMU(settings_);
    
    if ((errcode = imu_->IMUInit()) < 0) 
       return false; /* Failed to init IMU */
    
    if (imu_->getCompassCalibrationValid())
      Serial.println("Using compass calibration");
    else
      Serial.println("No valid compass calibration data found");  
  
    imu_->setSlerpPower(0.02);
    imu_->setGyroEnable(true);
    imu_->setAccelEnable(true);
    imu_->setCompassEnable(true);
}


bool Imu::read(protocol::imu &data)
{
  /* get the latest data if ready yet */
  if (imu_->IMURead()) 
  {                                
    RTIMU_DATA imu_data_raw = imu_->getIMUData();
    data.roll = imu_data_raw.fusionPose.x();
    data.pitch = imu_data_raw.fusionPose.y();
    data.yaw = imu_data_raw.fusionPose.z();
    data.accl_x = imu_data_raw.accel.x();
    data.accl_y = imu_data_raw.accel.y();
    data.accl_z = imu_data_raw.accel.z();
    data.gyro_x = imu_data_raw.gyro.x();
    data.gyro_y = imu_data_raw.gyro.y();
    data.gyro_z = imu_data_raw.gyro.z();
    data.mag_x = imu_data_raw.compass.x();
    data.mag_y = imu_data_raw.compass.y();
    data.mag_z = imu_data_raw.compass.z();
  
    return true;
  }
  return false;
}



