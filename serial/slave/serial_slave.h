#ifndef _SERIAL_SLAVE_H_INCLUDED
#define _SERIAL_SLAVE_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define GPS_NUM              1
#define ACCEL_NUM            2
#define GYRO_NUM             3
#define ACCEL_REQUEST_BYTE   6
#define GYRO_REQUEST_BYTE    6
#define ERR_NUM              9

#define GYRO_ADDR 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

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
} I2C_sensor_data;

class SerialSlave{
  public:
    void select_func(int send_byte);
	void send_GPS(void);
	void send_Accel(void);
	void send_Gyro(void);
	void send_all(void);
	void change_job(ring_buffer *);
	int saveSD(I2C_sensor_data);
};

extern SerialSlave Slave;

#endif
