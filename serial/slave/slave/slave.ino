//com12: slave
#include <Wire.h>
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"

#define START               '0'
#define RECEIVE             '1'

#define DATA_1              '1'
#define DATA_2              '2'
#define DATA_3              '3'

#define SLAVE_DEVICE_NUM     2
typedef struct {
  float f_data11;
  float f_data12;
  float f_data13;
} accel;

typedef union {
  accel f_data1;
  uint8_t data[sizeof(float)*3];
} test_u;

test_u tmp;
double gyroX, gyroY, gyroZ;
double accXval, accYval, accZval;

sensorData _data1;

void setup()
{  
  Serial.begin(9600);
  Serial.setintr(Slave.receive_data);
  IMU.sensorInit();
}

void loop()
{
  IMU.receiveAcc();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  gyroY   = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
  gyroZ   = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
  accXval = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  accZval = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  
  Slave.setData_Accel(accXval, accYval, accZval);
  Slave.setData_Gyro(gyroX, gyroY, gyroZ);
  interrupts();
  delay(100);
}


