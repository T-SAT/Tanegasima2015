#ifndef _GPS_H_INCLUDED_
#define _GPS_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SOFTWARE_SERIAL_RX       10
#define SOFTWARE_SERIAL_TX       11

#define GPS_SAMPLE_NUM         20

class GPS_Photo {
  public :
    void GPSInit(void);
    void recvGPS(float *flat, float *flon);
};

extern GPS_Photo gpsPhoto;

#endif
