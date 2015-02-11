#ifndef _SERIAL_SLAVE_H_INCLUDED
#define _SERIAL_SLAVE_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define GPS_NUM              (byte)1
#define ACCEL_NUM            (byte)2
#define GYRO_NUM             (byte)3

#define ACCEL_REQUEST_BYTE   (byte)6
#define GYRO_REQUEST_BYTE    (byte)6
#define ERR_NUM              (byte)9

#define GYRO_ADDR 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

#define START                (byte)0
#define RECIEVE              (byte)1

#define SLAVE_DEVICE_NUM           8

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
        static void select_func(void);
	static void change_job(ring_buffer *);
      	static void send_GPS(void);
        static void send_Accel(void);
	static void send_Gyro(void);
        static void setData_GPS(float flat, float flon, unsigned long int age);
        static void setData_Accel(int x, int y, int z);
        static void setData_Gyro(int x, int y, int z);
	int saveSD(sensorData data);
};

extern SerialSlave Slave;

#endif
