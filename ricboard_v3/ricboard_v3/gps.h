#ifndef GPS_H
#define GPS_H

#include <Adafruit_GPS.h>
#include "protocol.h"

class Gps
{

private:
	Adafruit_GPS gps_;
	uint32_t timer_;


public:

	void init();
	bool read(protocol::gps &data);
};
#endif //GPS_H
