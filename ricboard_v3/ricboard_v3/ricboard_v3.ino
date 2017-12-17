#include "ric_settings.h"
#include "ric_communicator.h"
#include "protocol.h"
#include "strober.h"
#include "timer.h"
#include "ultrasonic.h"
#include "imu.h"
#include "laser.h"
#include "gps.h"
#include <Servo.h>
/* this cpp acts as board manager */

/* DO NOT use Serial.print, because */
/* communicator use it for communication with pc */

Timer send_keepalive_timer, 
      get_keepalive_timer,
      send_readings_timer;
bool got_keepalive;
Strober strober;

/* front ultrasonic */
Servo servo;
Ultrasonic ultrasonic;
Imu imu;
Laser laser;
Gps gps;

/******************************************************/

void setup() 
{
  communicator::init(BAUDRATE);
  send_keepalive_timer.start(SEND_KA_INTERVAL);
  get_keepalive_timer.start(GET_KA_INTERVAL);
  send_readings_timer.start(SEND_READINGS_INTERVAL);

  /* torso servo */
  servo.attach(SERVO_PIN, SRVO_MIN, SRVO_MAX);
  servo.writeMicroseconds(SRVO_NEUTRAL);
  
  ultrasonic.init(ULTRASONIC_PIN);
  if (!imu.init())
    Serial.println("imu failed");//log("imu failed", protocol::logger::Code::ERROR);

  delay(5);

  laser.init();

  delay(5);

  gps.init();

  delay(5);

  /* pin mode must be invoked after i2c devices (arduino bug) */
  pinMode(INDICATOR_LED, OUTPUT);
  strober.setNotes(Strober::Notes::BLINK_SLOW);
}

/******************************************************/

void loop() 
{
  strober.play(INDICATOR_LED);
  keepAliveAndRead();  
  sendReadings();
}

/******************************************************/
void sendReadings()
{
  
  /* read IMU if available. IMU read must be called */
  /* as fast as possible (no delays)                */
  protocol::imu imu_pkg;
  bool valid_imu = false;
  if (imu.read(imu_pkg)) //if imu ready, send it
    valid_imu = true;
  
  if (send_readings_timer.finished())
  {
    /* ULTRASONIC */
    protocol::header ultrasonic_header;
    ultrasonic_header.type = protocol::Type:: ULTRASONIC;
    protocol::ultrasonic ultrasonic_pkg;
    ultrasonic_pkg.distance_mm = ultrasonic.readDistanceMm();
    communicator::ric::sendPkg(ultrasonic_header, sizeof(protocol::header));
    communicator::ric::sendPkg(ultrasonic_pkg, sizeof(protocol::ultrasonic));

    /* IMU */
    if (valid_imu)
    {
      protocol::header imu_header;
      imu_header.type = protocol::Type:: IMU;
      communicator::ric::sendPkg(imu_header, sizeof(protocol::header));
      communicator::ric::sendPkg(imu_pkg, sizeof(protocol::imu));
      
      //Serial.print("roll: "); Serial.println(imu_pkg.roll * 180 / M_PI);
      //Serial.print("pitch: "); Serial.println(imu_pkg.pitch * 180 / M_PI);
      //Serial.print("yaw: "); Serial.println(imu_pkg.yaw * 180 / M_PI);
    }

    /* LASER */
    uint16_t laser_read = laser.read();
    if (laser_read != (uint16_t)Laser::Code::ERROR)
    {
      protocol::header laser_header;
      laser_header.type = protocol::Type:: LASER;
      protocol::laser laser_pkg;
      laser_pkg.distance_mm = laser_read;
      communicator::ric::sendPkg(laser_header, sizeof(protocol::header));
      communicator::ric::sendPkg(laser_pkg, sizeof(protocol::laser));
    }

    /* GPS */
    protocol::gps gps_pkg;
    bool valid_gps = gps.read(gps_pkg);
    if (valid_gps)
    {
      protocol::header gps_header;
      gps_header.type = protocol::Type::GPS;
      communicator::ric::sendPkg(gps_header, sizeof(protocol::header));
      communicator::ric::sendPkg(gps_pkg, sizeof(protocol::gps));
    }

    send_readings_timer.startOver();
  }
}

/******************************************************/

void keepAliveAndRead()
{
  if (send_keepalive_timer.finished())
  {
    /* send keep alive */
    protocol::header ka_header;
    ka_header.type = protocol::Type:: KEEP_ALIVE;
    protocol::keepalive ka_pkg;
    communicator::ric::sendPkg(ka_header, sizeof(protocol::header));
    communicator::ric::sendPkg(ka_pkg, sizeof(protocol::keepalive));
    
    send_keepalive_timer.startOver();
  }

  if (get_keepalive_timer.finished())
  {
    if (got_keepalive) //connected to pc
    {
      strober.setNotes(Strober::Notes::STROBE);
      got_keepalive = false;
    }
    else //disconnected from pc
      strober.setNotes(Strober::Notes::BLINK_SLOW);
    get_keepalive_timer.startOver();
  }
  
  protocol::header incoming_header;
  if (communicator::ric::readPkg(incoming_header, sizeof(protocol::header)))
  {
    handleHeader(incoming_header);
  }
}

/******************************************************/

void handleHeader(const protocol::header &h)
{
    switch (h.type)
    {
        case protocol::Type::KEEP_ALIVE:
            protocol::keepalive ka_pkg;
            if (communicator::ric::readPkg(ka_pkg, sizeof(protocol::keepalive)))
            {
                //ka pkg is empty
                //log("got ka", 3);
                got_keepalive = true;
            }
            break;
        case protocol::Type::SERVO:
            protocol::servo servo_pkg;
            if (communicator::ric::readPkg(servo_pkg, sizeof(protocol::servo)))
            {
               servo.writeMicroseconds(servo_pkg.cmd);
               //log("got servo", servo_pkg.cmd);
            }
            break;
    }
}

/******************************************************/
void log(const char* msg_str, int32_t value)
{
    protocol::header logger_header;
    logger_header.type = protocol::Type::LOGGER;
    protocol::logger logger_pkg;
    strcpy(logger_pkg.msg, msg_str);
    
    logger_pkg.value = value;
    
    communicator::ric::sendPkg(logger_header, sizeof(protocol::header));
    communicator::ric::sendPkg(logger_pkg, sizeof(protocol::logger));
}
