#include "ric_communicator.h"

void RicCommunicator::sendKeepAlive()
{
  protocol::header keepalive_header;
  keepalive_header.type = (uint8_t)protocol::Type::KEEP_ALIVE;
  sendHeader(keepalive_header);
}

void RicCommunicator::sendHeader(const protocol::header &h)
{
  byte buff[sizeof(h)];
  memcpy(buff, &h, sizeof(h));
  send(buff, sizeof(h));
}

bool RicCommunicator::readHeader(protocol::header &h)
{
  byte buff[sizeof(h)];
  int bytes_read = read(buff, sizeof(h));
  if (bytes_read != sizeof(h))
    return false;
  memcpy(&h, buff, sizeof(h));
  return true;
}

