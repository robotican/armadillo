#include "ric_settings.h"
#include "communicator.h"
#include "protocol.h"
#include "strober.h"
#include "timer.h"
#include "ultrasonic.h"
#include "imu.h"
#include "laser.h"
#include "gps.h"
#include <Servo.h>
/* this cpp acts as board manager */

/* DO NOT use Serial.print, because              */
/* communicator use it for communication with pc */

Timer send_keepalive_timer, 
      get_keepalive_timer,
      send_readings_timer,
      ultrasonic_timer;
bool got_keepalive, send_data;
Strober strober;
Communicator com;
/* servo for controlling torso */
Servo servo;
Ultrasonic ultrasonic;
Imu imu;
Laser laser;
Gps gps;

byte pkg_buff[protocol::MAX_PKG_SIZE];

/******************************************************/

void setup() 
{
  com.init(BAUDRATE);
  Serial3.begin(9600);
  send_keepalive_timer.start(SEND_KA_INTERVAL);
  get_keepalive_timer.start(GET_KA_INTERVAL);
  send_readings_timer.start(SEND_READINGS_INTERVAL);
  ultrasonic_timer.start(UTLRASONICE_INTERVAL);

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
  
  readFromPC();
   
  checkKeepAliveFromPC();

  sendKeepAliveToPC(); 

   
  if (send_data)
    sendReadingsToPC();
}

/******************************************************/

void readFromPC()
{
  int pkg_type = com.read(pkg_buff); //read incoming pkgs
  if(pkg_type != -1) //incoming valid pkg
  {
    switch (pkg_type)
    {
      case (uint8_t)protocol::Type::KEEP_ALIVE:
      {
        strober.setNotes(Strober::Notes::STROBE);

        protocol::keepalive ka_pkg;
        Communicator::fromBytes(pkg_buff, sizeof(protocol::keepalive), ka_pkg);
        send_data = true;
        got_keepalive = true;
        break;
      }
      case (uint8_t)protocol::Type::SERVO:
      {
        protocol::servo servo_pkg;
        Communicator::fromBytes(pkg_buff, sizeof(protocol::servo), servo_pkg);
        servo.writeMicroseconds(servo_pkg.cmd);
        break;
      }
    }    
    clearBuffer(); //prepare buff for next pkg
  }
}

/******************************************************/

/******************************************************/

void sendKeepAliveToPC()
{
  if (send_keepalive_timer.finished() && send_data)
  {
    protocol::keepalive ka_pkg;    
    com.write(ka_pkg, sizeof(protocol::keepalive));
    send_keepalive_timer.startOver();
  }
}

/******************************************************/

void checkKeepAliveFromPC() 
{
  if (get_keepalive_timer.finished())
  {
    if (got_keepalive) //connected to pc
    {
      got_keepalive = false;
    }
    else //disconnected from pc
    {
      send_data = false;
      strober.setNotes(Strober::Notes::BLINK_SLOW);
    }
    get_keepalive_timer.startOver();
  }
}

/******************************************************/

void sendReadingsToPC()
{
  
  /* read IMU if available. IMU read must be called */
  /* as fast as possible (no delays)                */
  /*protocol::imu imu_pkg;
  bool valid_imu = false;
  if (imu.read(imu_pkg)) //if imu ready, send it
    valid_imu = true;*/
  
  if (send_readings_timer.finished())
  {
    /* ULTRASONIC */
    if (ultrasonic_timer.finished())
    {
       protocol::ultrasonic ultrasonic_pkg;
       if (ultrasonic.readDistanceMm(ultrasonic_pkg.distance_mm))
          com.write(ultrasonic_pkg, sizeof(protocol::ultrasonic));
       ultrasonic_timer.startOver();
    }
   

    /* IMU */
    /*if (valid_imu)
      com.write(imu_pkg, sizeof(protocol::imu));*/

    /*
    Serial.print("roll: "); Serial.println(imu_pkg.roll * 180 / M_PI);
    Serial.print("pitch: "); Serial.println(imu_pkg.pitch * 180 / M_PI);
    Serial.print("yaw: "); Serial.println(imu_pkg.yaw * 180 / M_PI);
    */

    /* LASER */
    uint16_t laser_read = laser.read();
    if (laser_read != (uint16_t)Laser::Code::ERROR)
    {
      protocol::laser laser_pkg;
      laser_pkg.distance_mm = laser_read;
      Serial3.println(laser_read);
      com.write(laser_pkg, sizeof(protocol::laser));
    }

    /* GPS */
    protocol::gps gps_pkg;
    bool valid_gps = gps.read(gps_pkg);
    if (valid_gps)
      com.write(gps_pkg, sizeof(protocol::gps));

    send_readings_timer.startOver();
  }
}

/******************************************************/
void log(const char* msg_str, int32_t value)
{
    protocol::logger logger_pkg;
    strcpy(logger_pkg.msg, msg_str);
    
    logger_pkg.value = value;
    com.write(logger_pkg, sizeof(protocol::logger));
}

/****************************************************/

void clearBuffer() 
{
  memset(pkg_buff, 0, protocol::MAX_PKG_SIZE);
}

