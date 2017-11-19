#ifndef RIC_COMMUNICATOR_H
#define RIC_COMMUNICATOR_H

#include <Arduino.h>
#include "communicator.h"
#include "protocol.h"

class RicCommunicator : public Communicator
{
  
private:

public:
  void sendKeepAlive();
  void sendHeader(const protocol::header &h);
  bool readHeader(protocol::header &h);

};

#endif //RIC_COMMUNICATOR_H
