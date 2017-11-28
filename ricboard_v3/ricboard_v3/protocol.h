#ifndef PROTOCOL_H
#define PROTOCOL_H

namespace protocol
{
  enum class Type
  {
    KEEP_ALIVE = 1,
    LOGGER = 2,
    ULTRASONIC = 3,
    LIDAR = 4
  };
  
  struct header
  {
    Type type;
  };
  
  struct package
  {
    
  };
  
  struct logger : package
  {
    char msg[128];
    uint8_t code;
  };
  
  struct sensor : package
  {
    
  };
  
  struct actuator : package
  {
  };

  struct ultrasonic : sensor
  {
    uint16_t distance_mm;
  };
  
  struct lidar : sensor
  {
    uint16_t distance_mm;  
  };
  
  struct elevator : actuator
  {
    uint16_t cmd_mm;  
  };
  
}



#endif //PROTOCOL_H
