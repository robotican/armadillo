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

	void init()
  {
    gps_ = Adafruit_GPS(&Serial2);
    gps_.begin(9600);
    gps_.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps_.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    gps_.sendCommand(PGCMD_ANTENNA);
    timer_ = millis();
  }
  
	bool read(protocol::gps &data)
  {
    /* read package from gps. each cycle read */
    /* one char into the packet. read should  */
    /* be invoked in loop                     */
  	gps_.read();
  	if (gps_.newNMEAreceived()) 
  	{
      if (!gps_.parse(gps_.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      	return false; // we can fail to parse a sentence in which case we should just wait for another
    }
    if (timer_ > millis()) timer_ = millis();
  
    /* adafruit recommend to read incoming data at 1Hz */
    if (millis() - timer_ > 1000) 
    {
      timer_ = millis(); // reset the timer
      if(!gps_.fix)
      	return false;
      
      data.lat = gps_.latitudeDegrees;
      data.lon = gps_.longitudeDegrees;
      data.lat_mark = gps_.lat;
      data.lon_mark = gps_.lon;
      data.elevation = gps_.altitude;
      data.hour = gps_.hour;
      data.minute = gps_.minute;
      data.seconds = gps_.seconds;
      data.year = gps_.year;
      data.month = gps_.month;
      data.day = gps_.day;
      data.speed = gps_.speed;      
      data.angle = gps_.angle;
      data.fix_quality = gps_.fixquality;
      data.satellites = gps_.satellites;
      data.fix = gps_.fix;
      return true;
    }
    return false;
   }
};
#endif //GPS_H


