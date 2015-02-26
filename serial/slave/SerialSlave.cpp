#include <Wire.h>
#include <SD.h>
#include "SerialSlave.h"
#include "SensorStick_9DoF.h"

SerialSlave Slave;
sensorData _data;

void SerialSlave::receive_data(ring_buffer *buf)
{
  char check;
  
  noInterrupts();
  check = Serial.read();
  interrupts();
  Serial.print(RECEIVE);
  Wire.begin(SLAVE_DEVICE_NUM);
  
  if(check == GPS_NUM || check == ACCEL_NUM || check == GYRO_NUM || check == ALL_NUM){
    switch(check){
    case GPS_NUM:
      Wire.onRequest(send_GPS);
      Serial.print(START);
      break;
  
    case ACCEL_NUM:
       IMU.receiveAcc();
      _data.Data.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
      _data.Data.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');
      _data.Data.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');
      Wire.onRequest(send_Accel);
      Serial.print(START);
      break;
    
    case GYRO_NUM:
      IMU.receiveGyro();
      _data.Data.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');
      _data.Data.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
      _data.Data.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
      Wire.onRequest(send_Gyro);
      Serial.print(START);
      break;
    
    case ALL_NUM:
      _data.Data.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');
      _data.Data.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
      _data.Data.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
      _data.Data.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
      _data.Data.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');
      _data.Data.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');
      Wire.onRequest(send_All);
      Serial.print(START);
      break;
      
    default :
      break; 
    }
  }
  
}

void SerialSlave::send_GPS(void)
{
  Wire.write(_data.Data.gps.byte_data, sizeof(_data.Data.gps.byte_data));
  Wire.begin();
}

void SerialSlave::send_Accel(void)
{ 
  Wire.write(_data.Data.accel.byte_data, sizeof(_data.Data.accel.byte_data));
  Serial.print(ENABLE);  
  Wire.begin();
}

void SerialSlave::send_Gyro(void)
{
  Wire.write(_data.Data.gyro.byte_data, sizeof(_data.Data.gyro.byte_data));
  Serial.print(ENABLE);
  Wire.begin();
}

void SerialSlave::send_All(void)
{
  Wire.write(_data.byte_data, sizeof(_data.byte_data));
  Serial.print(ENABLE);
  Wire.begin();
}

void SerialSlave::setData_GPS(float flat, float flon)
{
  _data.Data.gps.gps_data.flat = flat;
  _data.Data.gps.gps_data.flon = flon;
}

void SerialSlave::setData_Accel(float x, float y, float z)
{
  _data.Data.accel.float_data.xA = x;
  _data.Data.accel.float_data.yA = y;
  _data.Data.accel.float_data.zA = z;
}

void SerialSlave::setData_Gyro(float x, float y, float z)
{
  _data.Data.gyro.float_data.xG = x;
  _data.Data.gyro.float_data.yG = y;
  _data.Data.gyro.float_data.zG = z;
}

int SerialSlave::saveSD(sensorData data, unsigned long int time){
  File saveFile;
  static int initFlag=0;
  
  if(!SD.begin(4)) {
    return(-1);
  }
  
  saveFile = SD.open("data_log.txt", FILE_WRITE);
  
  if(saveFile) {
    if(initFlag == 0) {
      saveFile.println("xA,yA,zA,xG,yG,zG,LAT,LON,AGE,time");
      saveFile.close();
      initFlag = 1;
    }
    
    else {
      saveFile.print(data.Data.accel.float_data.xA);
      saveFile.print(",");
      saveFile.print(data.Data.accel.float_data.yA);
      saveFile.print(",");
      saveFile.print(data.Data.accel.float_data.zA);
      saveFile.print(",");
      saveFile.print(data.Data.gyro.float_data.xG);
      saveFile.print(",");
      saveFile.print(data.Data.gyro.float_data.yG);
      saveFile.print(",");
      saveFile.print(data.Data.gyro.float_data.zG);
      saveFile.print(",");
      saveFile.print(data.Data.gps.gps_data.flat);
      saveFile.print(",");
      saveFile.print(data.Data.gps.gps_data.flon);
      saveFile.print(",");
      saveFile.print(data.Data.gps.gps_data.flat);
      saveFile.print(",");
      saveFile.println(time);
      saveFile.close();
    }
  }
  
  else {
    return(-1);
  }
  
  return(0);
}

