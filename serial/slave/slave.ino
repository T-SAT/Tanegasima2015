//com12: slave
#include <Wire.h>
#include <SD.h>
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"

void setup()
{  
  Serial.begin(9600);
  Serial.setintr(Slave.receive_data);
  pinMode(10, OUTPUT);
  IMU.sensorInit();
}

void loop()
{
  sensorData tmp;
  unsigned long int time;
  int check_sd;
  
  IMU.receiveAcc();
  IMU.receiveGyro();
  time = millis();
  tmp.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  tmp.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
  tmp.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
  tmp.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  tmp.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  tmp.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  
  Slave.setData_Accel(tmp.accel.float_data.xA, tmp.accel.float_data.yA, tmp.accel.float_data.zA);
  Slave.setData_Gyro(tmp.gyro.float_data.xG, tmp.gyro.float_data.yG, tmp.gyro.float_data.zG);
  
  do {
    check_sd = Slave.saveSD(tmp, time);
  }while(check_sd == -1);
  
  interrupts();
  delay(100);
}


