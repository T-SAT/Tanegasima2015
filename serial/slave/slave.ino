//com12: slave
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"
#include <Wire.h>
#include <TinyGPS.h>
#include <SD.h>

void setup()
{
  Serial.begin(9600);
  Serial.setintr(Slave.change_job);
  IMU.sensorInit();
  Wire.begin();
}

void loop()
{
  double gyroX, gyroY, gyroZ;
  double accXval, accYval, accZval;
  
  IMU.receiveAcc();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  gyroY   = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
  gyroZ   = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
  accXval = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  accZval = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  
  //Serial.println(accXval);
  Slave.setData_Gyro(gyroX, gyroY, gyroZ);
  Slave.setData_Accel(accXval, accYval, accZval);
  interrupts();
}



