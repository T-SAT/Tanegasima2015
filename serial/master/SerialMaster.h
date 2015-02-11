#ifndef _MASTER_H_INCLUDED
#define _MASTER_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SLAVE_DEVICE_NUM     8

#define GPS_NUM              1
#define ACCEL_NUM            2
#define GYRO_NUM             3

#define ACCEL_REQUEST_BYTE   6
#define GYRO_REQUEST_BYTE    6
#define ERR_NUM              9

typedef struct {
  float flat;
  float flon;
} GPS;

typedef struct {
  int xA;
  int yA;
  int zA;
} Accel;

typedef struct {
  int xG;
  int yG;
  int zG;
} Gyro;

typedef struct {
  union {
    GPS float_data;
    uint8_t byte_data[8];
  } gps;
 
  union {
    Accel int_data;
    uint8_t byte_data[6];
  } accel;
  
  union {
    Gyro int_data;
    uint8_t byte_data[6];
  } gyro;
} sensorData;

class SerialMaster{
  public:
	void write_numbers(int , int);
	void request_data(int);
        
  private:
	sensorData _data;
};

extern SerialMaster Master;

#endif
