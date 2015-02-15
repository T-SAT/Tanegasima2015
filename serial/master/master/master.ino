//com10: master
#include <Wire.h>
#include "SerialMaster.h"

#define START                 '0'
#define RECEIVE               '1'

#define GPS_NUM               '1'
#define ACCEL_NUM             '2'
#define GYRO_NUM              '3'

#define SLAVE_DEVICE_NUM       2

typedef struct {
  float f_data1;
  float f_data2;
  float f_data3;
} accel;

typedef union {
  accel f_data;
  uint8_t data[sizeof(float)*3];
} test_u;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  Master.request_data(GYRO_NUM);
}

