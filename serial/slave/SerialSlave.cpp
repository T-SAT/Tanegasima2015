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
  
  if(check == GPS_NUM || check == ACCEL_NUM || check == GYRO_NUM){
    switch(check){
    case GPS_NUM:
      Wire.onRequest(send_GPS);
      Serial.print(START);
      break;
  
    case ACCEL_NUM:
      Wire.onRequest(send_Accel);
      Serial.print(START);
      break;
    
    case GYRO_NUM:
      Wire.onRequest(send_Gyro);
      Serial.print(START);
      break;
    
    case ALL_NUM:
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
  Wire.write(_data.gps.byte_data, sizeof(_data.gps.byte_data));
  Wire.begin();
  noInterrupts();
}

void SerialSlave::send_Accel(void)
{ 
  Wire.write(_data.accel.byte_data, sizeof(_data.accel.byte_data));
  Wire.begin();
  noInterrupts();
}

void SerialSlave::send_Gyro(void)
{
  Wire.write(_data.gyro.byte_data, sizeof(_data.gyro.byte_data));
  Wire.begin();
  noInterrupts();
}

void SerialSlave::send_All(void)
{
  Wire.write(_data.byte_data, sizeof(_data.byte_data));
  Wire.begin();
  noInterrupts();
}

void SerialSlave::setData_GPS(float flat, float flon, unsigned long int age)
{
  _data.gps.gps_data.flat = flat;
  _data.gps.gps_data.flon = flon;
  _data.gps.gps_data.age = age;
}

void SerialSlave::setData_Accel(float x, float y, float z)
{
  _data.accel.float_data.xA = x;
  _data.accel.float_data.yA = y;
  _data.accel.float_data.zA = z;
}

void SerialSlave::setData_Gyro(float x, float y, float z)
{
  _data.gyro.float_data.xG = x;
  _data.gyro.float_data.yG = y;
  _data.gyro.float_data.zG = z;
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
      saveFile.print(data.accel.float_data.xA);
      saveFile.print(",");
      saveFile.print(data.accel.float_data.yA);
      saveFile.print(",");
      saveFile.print(data.accel.float_data.zA);
      saveFile.print(",");
      saveFile.print(data.gyro.float_data.xG);
      saveFile.print(",");
      saveFile.print(data.gyro.float_data.yG);
      saveFile.print(",");
      saveFile.print(data.gyro.float_data.zG);
      saveFile.print(",");
      saveFile.print(data.gps.gps_data.flat);
      saveFile.print(",");
      saveFile.print(data.gps.gps_data.flon);
      saveFile.print(",");
      saveFile.print(data.gps.gps_data.flat);
      saveFile.print(",");
      saveFile.print(data.gps.gps_data.age);
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


