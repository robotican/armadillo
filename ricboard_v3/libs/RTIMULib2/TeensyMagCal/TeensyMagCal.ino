////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib2-Teensy
//
//  1. conncect teensy/ricboard to the computer with cable.
//  
//  2. upload this code to the board and open serial port
//
//  3. look at the serial port and rotate the imu on all of the axes,
//     at least 360 degrees on each direction for each one of the axes
//     until the nunbers are not changeing anymore.
// 
//  4. when the nunber stopped change, press 's' (lowercase)
//     and send (in the serial monitor).
//
//
//



#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMULib.h"
#include "RTIMUMagCal.h"

RTIMU *imu;                                           // the IMU object
RTIMUSettings *settings;                              // the settings object
RTIMUMagCal *magCal;                                  // the mag calibration object
unsigned long lastDisplay;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  200                         // interval between min/max displays

void setup()
{
    int errcode;

    Serial.begin(SERIAL_PORT_SPEED);
    while (!Serial) {
        ; // wait for serial port to connect. 
    }
    Wire.begin();
   
    settings = new RTIMUSettings();
    imu = RTIMU::createIMU(settings);                        // create the imu object
  
    if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
    
    Serial.print("TeensyMagCal starting using device "); Serial.println(imu->IMUName());
    Serial.println("Enter s to save current data to SD card");

    imu->setCompassCalibrationMode(true);
    magCal = new RTIMUMagCal(settings);
    magCal->magCalInit();
    lastDisplay = millis();
}

void loop()
{  
    unsigned long now = millis();
  
    if (imu->IMURead()) {                                 // get the latest data
        magCal->newMinMaxData(imu->getIMUData().compass);
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
            Serial.println("-------");
            Serial.print("minX: "); Serial.print(magCal->m_magMin.data(0));
            Serial.print(" maxX: "); Serial.print(magCal->m_magMax.data(0)); Serial.println();
            Serial.print("minY: "); Serial.print(magCal->m_magMin.data(1));
            Serial.print(" maxY: "); Serial.print(magCal->m_magMax.data(1)); Serial.println();
            Serial.print("minZ: "); Serial.print(magCal->m_magMin.data(2));
            Serial.print(" maxZ: "); Serial.print(magCal->m_magMax.data(2)); Serial.println();
        }
    }
  
    if (Serial.available()) {
        if (Serial.read() == 's') {                  // save the data
            magCal->magCalSaveMinMax();
            Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
            while(1) ;
        }
    }
}
