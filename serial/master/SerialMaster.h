#ifndef _MASTER_H_INCLUDED
#define _MASTER_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SLAVE_DEVICE_NUM           8

#define GPS_NUM              (byte)1
#define ACCEL_NUM            (byte)2
#define GYRO_NUM             (byte)3

#define ACCEL_REQUEST_BYTE         6
#define GYRO_REQUEST_BYTE          6
#define ERR_NUM                    9

#define START                (byte)0
#define RECIEVE              (byte)1

typedef struct {
  float flat;
  float flon;
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
    GPS float_data;
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
	void request_data(int);
        void trans_start();
  private:
	sensorData _data;
};

extern SerialMaster Master;

#endif
