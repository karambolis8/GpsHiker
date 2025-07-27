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

NMEAGPS  gps;
gps_fix  fix;

struct GpsReadouts 
{
  bool gpsHasFix;
  bool wasGpsFix;
  long speed;
  long maxSpeed;
  int year;
  int month;
  int day;
  int hour;
  int minutes;
  int numSV;
  float heading;
  long latitude;
  long longitude;
  char latChar;
  char lonChar;
  int altitude;
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

  if (gps.available(gpsPort)) 
  {
    fix = gps.read();

    gpsReadouts->numSV = gps.sat_count;

    // if(gpsReadouts->numSV > GPS_MIN_SAT && gps.sat_count > GPS_MIN_SAT)
    // {
    //   gpsReadouts->gpsHasFix = true;
    //   gpsReadouts->wasGpsFix = true;
    // }
    // else if(gpsReadouts->numSV < GPS_MIN_SAT && gps.sat_count > GPS_MIN_SAT)
    // {
    //   gpsReadouts->gpsHasFix = true;
    //   gpsReadouts->wasGpsFix = true;
    // }
    // else if(gpsReadouts->numSV > GPS_MIN_SAT && gps.sat_count < GPS_MIN_SAT)
    // {
    //   gpsReadouts->gpsHasFix = false;
    //   gpsReadouts->wasGpsFix = true;
    // }
    // else
    // {
    //   gpsReadouts->gpsHasFix = false;
    // }

    // if(fix.valid.speed)
    // {
    //   gpsReadouts->speed = fix.speed_kph();
    //   if(gpsReadouts->speed > gpsReadouts->maxSpeed)
    //   {
    //     gpsReadouts->maxSpeed = gpsReadouts->speed;
    //   }
    // }

    // if(fix.valid.date)
    // {
    //   gpsReadouts->year = (int)fix.dateTime.year + 2000;
    //   gpsReadouts->month = (int)fix.dateTime.month;
    //   gpsReadouts->day = (int)fix.dateTime.date;
    // }

    // if(fix.valid.time)
    // {
    //   gpsReadouts->hour = (int)fix.dateTime.hours + TIMEZONE_OFFSET;
    //   gpsReadouts->minutes = (int)fix.dateTime.minutes;

    //   if(SUMMER_TIME)
    //   {
    //     gpsReadouts->hour += 1;
    //   }
    // }

    // if(fix.valid.location)
    // {
    //   gpsReadouts->latitude = fix.latitude();
    //   gpsReadouts->longitude = fix.longitude();
    //   // gpsReadouts->latChar = gpsReadouts->fix.longitudeDMS.EW(); // not working. maybe library to update
    //   // gpsReadouts->lonChar = gpsReadouts->fix.latitudeDMS.NS();

    //   //distance calculation basing on precise longitudeL and latitudeL
    //   //https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Data%20Model.md#usage
    //   //https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Location.md
    //   //https://github.com/SlashDevin/NeoGPS/issues/15
    //   //calculating time since reset
    //   //calculating average speed basing on distance and time
    //   //calculating walking speed without stops
    // }

    // if(fix.valid.heading)
    // {
    //   gpsReadouts->heading = fix.heading();
    // }

    // if(fix.valid.altitude)
    // {
    //   gpsReadouts->altitude = fix.alt.whole;
    // }
  }

  gpsReadouts->lastReadout = millis();
}