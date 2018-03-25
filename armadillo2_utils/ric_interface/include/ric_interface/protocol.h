/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elchay Rauper*/

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

namespace ric_interface
{
    namespace protocol
    {
        enum class Type
        {
            KEEP_ALIVE = 100,
            LOGGER = 101,
            ULTRASONIC = 102,
            LASER = 103,
            IMU = 104,
            GPS = 105,
            SERVO = 106,
            EMERGENCY_ALARM = 107,
            ERROR = 108
        };

        enum class ErrCode
        {
            INIT = 50,
            READ = 51,
            CALIB = 52
        };

        const uint16_t MAX_PKG_SIZE = 512;
        const uint8_t HEADER_CODE = 200;
        const uint8_t HEADER_INDX = 0;
        const uint8_t PKG_SIZE_INDX = 1;

        struct package
        {
            uint8_t type = 0;
            uint8_t id = 0;
        };

        struct keepalive : package
        {
            keepalive() { type = (uint8_t)Type::KEEP_ALIVE; }
        };

        struct logger : package
        {
            logger() { type = (uint8_t)Type::LOGGER; }
            char msg[30];
            int32_t value = 0;
        };

        struct error : package
        {
            uint8_t code = 0; //error code
            uint8_t comp_type = 0; //reporting component type
            uint8_t comp_id = 0; //reporting component id
            error() { type = (uint8_t)Type::ERROR; }
        };

        struct sensor : package
        {

        };

        struct actuator : package
        {
        };

        struct emergency_alarm : sensor
        {
            emergency_alarm() { type = (uint8_t)Type::EMERGENCY_ALARM; }
            bool is_on = false;
        };

        struct ultrasonic : sensor
        {
            ultrasonic() { type = (uint8_t)Type::ULTRASONIC; }
            uint16_t distance_mm = 0;
        };

        struct laser : sensor
        {
            laser() { type = (uint8_t)Type::LASER; }
            uint16_t distance_mm = 0;
        };

        struct imu : sensor
        {
            imu() { type = (uint8_t)Type::IMU; }
            float roll_rad = 0,
                    pitch_rad = 0,
                    yaw_rad = 0,
                    accl_x_rad = 0,
                    accl_y_rad = 0,
                    accl_z_rad = 0,
                    gyro_x_rad = 0,
                    gyro_y_rad = 0,
                    gyro_z_rad = 0,
                    mag_x_rad = 0,
                    mag_y_rad = 0,
                    mag_z_rad = 0;
        };

        struct gps : sensor
        {
            gps() { type = (uint8_t)Type::GPS; }
            double lat = 0, lon = 0;
            float alt = 0;
            float speed = 0;
            float heading = 0;
            float satellites = 0;
            uint8_t date_time = 0; //UTC hundredths of a second
        };

        struct servo : actuator
        {
            servo() { type = (uint8_t)Type::SERVO; }
            uint16_t cmd = 0; //servo command 1000-2000
        };
    }
}



#endif //PROTOCOL_H