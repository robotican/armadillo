#ifndef PROTOCOL_H
#define PROTOCOL_H

namespace protocol
{
  enum class Type
  {
    KEEP_ALIVE = 1
  };
  
  struct header
  {
    uint8_t type;
  };
  
  struct package
  {
    
  };
  
  struct keep_alive : package
  {
    
  };
}



#endif //PROTOCOL_H
