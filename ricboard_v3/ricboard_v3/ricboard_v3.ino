#include "ric_settings.h"
#include "ric_communicator.h"
#include "protocol.h"
#include "strober.h"
#include "timer.h"
#include "ultrasonic.h"

/* this cpp acts as board manager */
Timer send_keepalive_timer, 
      get_keepalive_timer,
      send_readings_timer;
Strober strober;
bool got_keepalive;

/* front ultrasonic */
Ultrasonic ultrasonic;

/******************************************************/

void setup() 
{
  pinMode(INDICATOR_LED, OUTPUT);
  strober.setNotes(Strober::Notes::BLINK_SLOW);
  communicator::init(BAUDRATE);
  send_keepalive_timer.start(SEND_KA_INTERVAL);
  get_keepalive_timer.start(GET_KA_INTERVAL);
  send_readings_timer.start(SEND_READINGS_INTERVAL);

  ultrasonic.init(ULTRASONIC_PIN);
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
  if (send_readings_timer.finished())
  {
    /* read ultrasonic */
    protocol::ultrasonic ultrasonic_pkg;
    ultrasonic_pkg.distance_mm = ultrasonic.readDistanceMm();
    communicator::ric::sendUltrasonic(ultrasonic_pkg);

    log("hello world", 2);
    
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

void log(const char* msg_str, uint8_t code)
{
  //Serial.println(msg_str);
    protocol::logger logger_pkg;
    strcpy(logger_pkg.msg, msg_str);
    
    logger_pkg.code = code;
    communicator::ric::sendLogger(logger_pkg);
}
