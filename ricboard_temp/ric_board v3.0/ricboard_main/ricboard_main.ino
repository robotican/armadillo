#include "communicator.h"
#include "protocol.h"

#define BAUDRATE 115200

Communicator comm;

void setup() 
{
  comm.init(BAUDRATE);
}

void loop() 
{
  //protocol::header pkg_header;
  //byte buff[sizeof(pkg_header)];
  //comm.read(buff, sizeof(pkg_header));
  //memcpy(buff, &pkg_header, sizeof(pkg_header));
  //Serial.println((int)pkg_header.type);


  protocol::header pkg_header;
  pkg_header.type = 50;
  //byte* buff = reinterpret_cast<byte*>(&pkg_header);
  byte buff[sizeof(pkg_header)];
  memcpy(buff, &pkg_header, sizeof(pkg_header));
  comm.send(buff, sizeof(pkg_header));
  Serial.write(7);
 // delay(100);
}
