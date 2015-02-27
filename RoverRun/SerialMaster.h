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
#define ALL_NUM                   '4'

#define ACCEL_REQUEST_BYTE         12
#define GYRO_REQUEST_BYTE          12
#define ERR_NUM                    9

#define START                '0'
#define RECEIVE              '1'
#define ENABLE               '2'

#define SD_PIN                8

typedef struct {
  float flat;
  float flon;
} 
GPS;

typedef struct {
  float xA;
  float yA;
  float zA;
} 
Accel;

typedef struct {
  float xG;
  float yG;
  float zG;
} 
Gyro;

typedef union {
  uint8_t byte_data[32];
  struct {
    union {
      GPS gps_data;
      uint8_t byte_data[8];
    } 
    gps;

    union {
      Accel float_data;
      uint8_t byte_data[12];
    } 
    accel;

    union {
      Gyro float_data;
      uint8_t byte_data[12];
    } 
    gyro;
  } 
  Data;  
} 
sensorData;

class SerialMaster{
public :
  SerialMaster();
  void write_numbers(int , int);
  void request_data(char);
  float get(char sensor, char axis);

public :
  int saveData(sensorData data, unsigned long int time);
  int saveLog(char *str, float data, unsigned long int time);
  int saveLog(float data);

private :
  sensorData _data;
};

extern SerialMaster Master;

#endif




