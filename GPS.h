#include "NMEAGPS.h"
#include "GPSport.h"
#include "NeoTime.h"

#ifndef NMEAGPS_PARSE_GSV
#error You must define NMEAGPS_PARSE_GSV in NMEAGPS_cfg.h!
#endif

#ifndef NMEAGPS_PARSE_SATELLITES
#error You must define NMEAGPS_PARSE_SATELLITE in NMEAGPS_cfg.h!
#endif

#ifndef NMEAGPS_PARSE_SATELLITE_INFO
#error You must define NMEAGPS_PARSE_SATELLITE_INFO in NMEAGPS_cfg.h!
#endif

struct GpsReadouts 
{
  NMEAGPS  gps;
  gps_fix  fix;
  long speed;
  long maxSpeed;
  int year;
  int month;
  int day;
  int hour;
  int minutes;
  int numSV;
  int gpsHasFix;
  bool wasGpsFix;
  unsigned long lastReadout;
};

void initGPS()
{
  gpsPort.begin(GPS_BAUD);
}

void readGPS(GpsReadouts* gpsReadouts)
{
  long readStart = millis();

  if(readStart - gpsReadouts->lastReadout < GPS_READ_DELAY)
  {
    return; // Avoid reading too often
  }

  gpsReadouts->lastReadout = readStart;

  if (gpsReadouts->gps.available( gpsPort )) 
  {
    gpsReadouts->fix = gpsReadouts->gps.read();

    Serial.println(gpsReadouts->fix.latitude());
    Serial.println(gpsReadouts->fix.longitude());
    Serial.println(gpsReadouts->fix.alt.whole);
    Serial.println(gpsReadouts->fix.heading_cd());
    Serial.println(gpsReadouts->fix.satellites);
    Serial.println(gpsReadouts->gps.sat_count);

    if(gpsReadouts->numSV > GPS_MIN_SAT && gpsReadouts->gps.sat_count > GPS_MIN_SAT)
    {
      gpsReadouts->gpsHasFix = true;
      gpsReadouts->wasGpsFix = true;
    }
    else if(gpsReadouts->numSV < GPS_MIN_SAT && gpsReadouts->gps.sat_count > GPS_MIN_SAT)
    {
      gpsReadouts->gpsHasFix = true;
      gpsReadouts->wasGpsFix = true;
    }
    else if(gpsReadouts->numSV > GPS_MIN_SAT && gpsReadouts->gps.sat_count < GPS_MIN_SAT)
    {
      gpsReadouts->gpsHasFix = false;
      gpsReadouts->wasGpsFix = true;
    }
    else
    {
      gpsReadouts->gpsHasFix = false;
    }

    gpsReadouts->numSV = gpsReadouts->gps.sat_count;

    if(gpsReadouts->fix.valid.speed)
    {
      gpsReadouts->speed = gpsReadouts->fix.speed_kph();
      if(gpsReadouts->speed > gpsReadouts->maxSpeed)
      {
        gpsReadouts->maxSpeed = gpsReadouts->speed;
      }
    }
    if(gpsReadouts->fix.valid.date)
    {
      gpsReadouts->year = (int)gpsReadouts->fix.dateTime.year + 2000;
      gpsReadouts->month = (int)gpsReadouts->fix.dateTime.month;
      gpsReadouts->day = (int)gpsReadouts->fix.dateTime.date;
    }
    if(gpsReadouts->fix.valid.time)
    {
      gpsReadouts->hour = (int)gpsReadouts->fix.dateTime.hours + TIMEZONE_OFFSET;
      gpsReadouts->minutes = (int)gpsReadouts->fix.dateTime.minutes;

      if(SUMMER_TIME)
      {
        gpsReadouts->hour += 1;
      }
    }
  }

  Serial.print("GPS read time: ");
  Serial.print(millis() - readStart);
  Serial.println("ms");
}