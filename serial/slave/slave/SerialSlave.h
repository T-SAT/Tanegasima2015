#ifndef _SERIAL_SLAVE_H_INCLUDED
#define _SERIAL_SLAVE_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define GPS_NUM                   '1'
#define ACCEL_NUM                 '2'
#define GYRO_NUM                  '3'

#define ACCEL_REQUEST_BYTE        12
#define GYRO_REQUEST_BYTE         12
#define ERR_NUM                    9

#define GYRO_ADDR 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

#define START                     '0'
#define RECEIVE                   '1'

#define SLAVE_DEVICE_NUM           2

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

class SerialSlave{
  public:
        static void receive_data(ring_buffer *buf);
      	static void send_GPS(void);
        static void send_Accel(void);
	static void send_Gyro(void);
        static void setData_GPS(float flat, float flon, unsigned long int age);
        static void setData_Accel(float x, float y, float z);
        static void setData_Gyro(float x, float y, float z);
	int saveSD(sensorData data);
};

extern SerialSlave Slave;

#endif
