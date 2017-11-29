#include "ric_settings.h"
#include "ric_communicator.h"
#include "protocol.h"
#include "strober.h"
#include "timer.h"
#include "ultrasonic.h"
#include "imu.h"
#include "laser.h"

/* this cpp acts as board manager */

Timer send_keepalive_timer, 
      get_keepalive_timer,
      send_readings_timer;
bool got_keepalive;
Strober strober;

/* front ultrasonic */
Ultrasonic ultrasonic;
Imu imu;
Laser laser;

/******************************************************/

void setup() 
{
  
  communicator::init(BAUDRATE);
  send_keepalive_timer.start(SEND_KA_INTERVAL);
  get_keepalive_timer.start(GET_KA_INTERVAL);
  send_readings_timer.start(SEND_READINGS_INTERVAL);

  //delay(200);
  
  ultrasonic.init(ULTRASONIC_PIN);
  if (!imu.init())
    Serial.println("imu failed");//log("imu failed", protocol::logger::Code::ERROR);

  delay(200);

  laser.init();

  delay(200);

  /* pin mode must be invoked after i2c devices (arduino bug) */
  pinMode(INDICATOR_LED, OUTPUT);
  strober.setNotes(Strober::Notes::BLINK_SLOW);
  
  delay(200);
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
  /* as fast as possible (no delays*                */
  protocol::imu imu_pkg;
  bool valid_imu = false;
  if (imu.read(imu_pkg)) //if imu ready, send it
    valid_imu = true;
  
     
  if (send_readings_timer.finished())
  {
    /* ULTRASONIC */
    protocol::ultrasonic ultrasonic_pkg;
    ultrasonic_pkg.distance_mm = ultrasonic.readDistanceMm();
    communicator::ric::sendUltrasonic(ultrasonic_pkg);

    /* IMU */
    if (valid_imu)
    {
      communicator::ric::sendImu(imu_pkg);
        //Serial.print("roll: "); Serial.println(imu_pkg.roll * 180 / M_PI);
        //Serial.print("pitch: "); Serial.println(imu_pkg.pitch * 180 / M_PI);
        //Serial.print("yaw: "); Serial.println(imu_pkg.yaw * 180 / M_PI);
    }

    /* LASER */
    uint16_t laser_read = laser.read();
    if (laser_read != (uint16_t)Laser::Code::ERROR)
    {
      protocol::laser laser_pkg;
      laser_pkg.distance_mm = laser_read;
      communicator::ric::sendLaser(laser_pkg);
    }

    
    send_readings_timer.startOver();
  }
}

/******************************************************/

void keepAliveAndRead()
{
  if (send_keepalive_timer.finished())
  {
    communicator::ric::sendKeepAlive();
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
  if (communicator::ric::readHeader(incoming_header))
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
            got_keepalive = true;
            break;
    }
}

/******************************************************/
void log(const char* msg_str, protocol::logger::Code code)
{
    protocol::logger logger_pkg;
    strcpy(logger_pkg.msg, msg_str);
    
    logger_pkg.code = code;
    communicator::ric::sendLogger(logger_pkg);
}
