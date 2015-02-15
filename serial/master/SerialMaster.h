//com10: master
#ifndef _MASTER_H_INCLUDED
#define _MASTER_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SLAVE_DEVICE_NUM           2

#define GPS_NUM                   '1'
#define ACCEL_NUM                 '2'
#define GYRO_NUM                  '3'

#define ACCEL_REQUEST_BYTE         12
#define GYRO_REQUEST_BYTE          12
#define ERR_NUM                    9

#define START                '0'
#define RECEIVE              '1'

typedef struct {
  float flat;
  float flon;
  unsigned long int age;
} GPS;

typedef struct {
  float xA;
  float yA;
  float zA;
} Accel;

typedef struct {
  float xG;
  float yG;
  float zG;
} Gyro;

typedef struct {
  union {
    GPS gps_data;
    uint8_t byte_data[8];
  } gps;
 
  union {
    Accel float_data;
    uint8_t byte_data[12];
  } accel;
  
  union {
    Gyro float_data;
    uint8_t byte_data[12];
  } gyro;
  
} sensorData;

class SerialMaster{
  public:
	void write_numbers(int , int);
	void request_data(char);
  private:
	sensorData _data;
};

extern SerialMaster Master;

#endif
