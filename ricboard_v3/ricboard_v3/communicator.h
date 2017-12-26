#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <Arduino.h>
#include "protocol.h"
#include "crc8.h"

class Communicator
{

  public:
  
  enum State 
  {
    HEADER_PART_A,
    HEADER_PART_B,
    PACKAGE, 
    CHECKSUM
  };
  
  void init(int baudrate)
  {
    Serial.begin(baudrate);
  }

  /* return -1 for reading in process, or type of the incoming pkg */
  int read(byte buff[]) 
  {
    switch (state_)
    {
      case HEADER_PART_A: //read header
      {
        if (tryReadHeader())
          state_ = HEADER_PART_B;
        break;
      }
      case HEADER_PART_B: //read pkg size
      {
        pkg_size_ = tryReadPkgSize();
        if (pkg_size_ != -1)
          state_ = PACKAGE;
        break;
      }
      case PACKAGE:
      {
        int incoming = Serial.read();
        if (incoming != -1)
          buff[pkg_indx_++] = (byte)incoming;

        if (pkg_indx_ >= pkg_size_) //done reading pkg content
         state_ = CHECKSUM;
        break;
      }
      case CHECKSUM:
      {
        int incoming = Serial.read();
        if (incoming != -1)
        {
          byte incoming_checksum = (byte)incoming;
          byte computed_checksum = crc_.get_crc(buff, pkg_size_);
          if (incoming_checksum == computed_checksum)
          {
            protocol::package pkg;
            fromBytes(buff, sizeof(protocol::package), pkg);
            reset();
            return (uint8_t)pkg.type;
          }
          else
            return -2; //wrong checksum
        }
        break;
      }
    }
    return -1; //still reading
  }

  static void fromBytes(byte buff[], size_t pkg_size, protocol::package &pkg)
  {
    memcpy(&pkg, buff, pkg_size);
  }

  static void toBytes(const protocol::package &pkg, size_t pkg_size, byte buff[])
  {
    memcpy(buff, &pkg, pkg_size);
  }

  /* sending: |header|size|pkg bytes|checksum| */
  bool write(const protocol::package &pkg, size_t pkg_size)
  {
      /* send pkg header */
      byte header_buff[2];
      header_buff[protocol::HEADER_INDX] = protocol::HEADER_CODE;
      header_buff[protocol::PKG_SIZE_INDX] = pkg_size;
      if (Serial.write(header_buff, 2) != 2)
          return false;
      byte pkg_buff[pkg_size];
      toBytes(pkg, pkg_size, pkg_buff);
      byte checksum = crc_.get_crc(pkg_buff, pkg_size);
      /* send pkg content and the checksum */
      if (Serial.write(pkg_buff, pkg_size) != pkg_size)
          return false;
      Serial.write(checksum);
      return true;
  }


  private:
  State state_ = HEADER_PART_A;
  int pkg_indx_ = 0;
  int pkg_size_ = 0;
  Crc8 crc_;
  
  void reset()
  {
    state_ = HEADER_PART_A;
    pkg_indx_ = 0;
    pkg_size_ = 0;
  }
  
  /* try to read valid header start */
  bool tryReadHeader()
  {
    if (Serial.available()>0 && Serial.read() == protocol::HEADER_CODE)
       return true;
    return false;
  }

  /* return -1 for failure and pkg size as success */
  int tryReadPkgSize()
  {
    if (Serial.available()>0)
      return Serial.read();
    return -1;
  }
};
    
#endif //COMMUNICATOR_H
