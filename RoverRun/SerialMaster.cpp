#include <Wire.h>
#include <SD.h>
#include "SerialMaster.h"

SerialMaster Master;
File dataFile;
File logFile;

SerialMaster::SerialMaster(void)
{

}

void SerialMaster::write_numbers(int numbers, int dev_number)
{
  Wire.beginTransmission(dev_number);
  Wire.write(numbers);
  Wire.endTransmission();
  delay(100);
}

void SerialMaster::request_data(char select_num)
{
  int i=0;
  byte check = '0';
  unsigned long int time, time1;

  time = millis();
  while(Serial.read() != ENABLE){
    if(5000 < millis() - time)
      break;
  }

  if(select_num == GPS_NUM || select_num == ALL_NUM){
    time = millis();
    while(check != RECEIVE){
      Serial.print(select_num);
      for(time1 = millis(); millis() - time1 < 1500;)
        check = Serial.read();
      if(1000 < millis() - time)
        break; 
    }
  }

  else {
    time = millis();
    while(check != RECEIVE){
      Serial.print(select_num);
      delay(70);
      check = Serial.read();
      if(1500 < millis() - time)
        break; 
    }
  }

  time = millis();
  while(check != START){
    check = Serial.read();
    if(2500 < millis() - time)
      break;
  }

  switch(select_num){
  case ACCEL_NUM:
    time = millis();
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      delay(1500);
      if(1500 < millis() - time)
        break;
    }

    time = millis();
    while(Wire.available()){
      _data.Data.accel.byte_data[i] = Wire.read();
      i++;
      if(5000 < millis() - time)
        break;
    }

    break;

  case GYRO_NUM:
    time = millis();
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      delay(1500);
      if(5000 < millis() - time)
        break;
    }

    time = millis();
    while(Wire.available()){
      _data.Data.gyro.byte_data[i] = Wire.read();
      i++;
      if(5000 < millis() - time)
        break;
    }
    break;

  case GPS_NUM:
    time = millis();
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*2);
      delay(3000);
      if(5000 < millis() - time)
        break;
    }

    time = millis();
    while(Wire.available()){
      _data.Data.gps.byte_data[i] = Wire.read();
      i++;
      if(5000 < millis() - time)
        break;
    }
    break;

  case ALL_NUM:
    time = millis();
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*8);
      delay(1500);
      if(5000 < millis() - time)
        break;
    }

    time = millis();
    while(Wire.available()){
      _data.byte_data[i] = Wire.read();
      i++;
      if(5000 < millis() - time)
        break;
    }
    break;

  default :
    break;
  }  

}

float SerialMaster::get(char sensor, char axis)
{
  switch(sensor){
  case ACCEL_NUM:
    switch(axis){
    case 'x':
      return(_data.Data.accel.float_data.xA);
      break;

    case 'y':
      return(_data.Data.accel.float_data.yA);
      break;

    case 'z':
      return(_data.Data.accel.float_data.zA);
      break;

    default:
      return(0);
      break;
    }
    return(0);
    break;

  case GYRO_NUM:
    switch(axis){
    case 'x':
      return(_data.Data.gyro.float_data.xG);
      break;

    case 'y':
      return(_data.Data.gyro.float_data.yG);
      break;

    case 'z':
      return(_data.Data.gyro.float_data.zG);
      break;

    default:
      return(0);
      break;
    }

  case GPS_NUM:
    switch(axis){
    case 'x':
      return(_data.Data.gps.gps_data.flat);
      break;

    case 'y':
      return(_data.Data.gps.gps_data.flon);
      break;

    default:
      return(0);
      break;
    }

  default:
    return(0);
    break;
  }

}

int SerialMaster::saveData(sensorData data)
{
  static int initFlag=0;

  dataFile = SD.open("Dlog.txt", FILE_WRITE);

  if(dataFile) {
    if(initFlag == 0) {
      dataFile.println("xA,yA,zA,xG,yG,zG,LAT,LON,AGE,time");
      dataFile.close();
      initFlag = 1;
    }

    else {
      dataFile.print(data.Data.accel.float_data.xA);
      dataFile.print(",");
      dataFile.print(data.Data.accel.float_data.yA);
      dataFile.print(",");
      dataFile.print(data.Data.accel.float_data.zA);
      dataFile.print(",");
      dataFile.print(data.Data.gyro.float_data.xG);
      dataFile.print(",");
      dataFile.print(data.Data.gyro.float_data.yG);
      dataFile.print(",");
      dataFile.print(data.Data.gyro.float_data.zG);
      dataFile.print(",");
      dataFile.print(data.Data.gps.gps_data.flat);
      dataFile.print(",");
      dataFile.print(data.Data.gps.gps_data.flon);
      dataFile.print(",");
      dataFile.print(data.Data.gps.gps_data.flat);
      dataFile.print(",");
      dataFile.print(millis());
      dataFile.println();
      dataFile.close();
    }
  }

  else {
    return(-1);
  }

  return(0);
}

int SerialMaster::saveLog(char *str, float data)
{

  logFile = SD.open("Clog.txt", FILE_WRITE);

  if(logFile){
    logFile.print(str); 
    logFile.print(",");
    logFile.print(data);
    logFile.print(",");
    logFile.print(millis());
    logFile.println();
    logFile.close();
  }

  else {
    return(-1);
  }

  return(0);
}

