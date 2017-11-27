#include "ric_settings.h"

/* this cpp acts as board manager */

RicCommunicator comm;
Timer send_keepalive_timer, get_keepalive_timer;
Strober strober;
bool got_keepalive;

/******************************************************/
void setup() 
{
  pinMode(INDICATOR_LED, OUTPUT);
  strober.setNotes(Strober::Notes::BLINK_SLOW);
  comm.init(BAUDRATE);
  send_keepalive_timer.start(SEND_KA_INTERVAL);
  get_keepalive_timer.start(GET_KA_INTERVAL);
}
/******************************************************/
void loop() 
{
  strober.play(INDICATOR_LED);
  keepAliveAndRead();  
  
}
/******************************************************/
void keepAliveAndRead()
{
  if (send_keepalive_timer.finished())
  {
    comm.sendKeepAlive();
    send_keepalive_timer.startOver();
  }

  if (get_keepalive_timer.finished())
  {
    if (got_keepalive)
    {
      strober.setNotes(Strober::Notes::STROBE);
      got_keepalive = false;
    }
    else
      strober.setNotes(Strober::Notes::BLINK_SLOW);
    get_keepalive_timer.startOver();
  }

  protocol::header incoming_header;
  if (comm.readHeader(incoming_header))
  {
    handleHeader(incoming_header);
  }
}
/******************************************************/
void handleHeader(const protocol::header &h)
{
  Serial.write(h.type);
    switch (h.type)
    {
        case (int)protocol::Type::KEEP_ALIVE:
            got_keepalive = true;
            break;
    }
}
/******************************************************/

