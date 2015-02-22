#include <Wire.h>
#include <SD.h>
#include "SerialMaster.h"

SerialMaster Master;
File saveFile;

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
  
  while(check != RECEIVE){
    Serial.print(select_num);
    delay(60);
    check = Serial.read();
  }
  
  while(check != START)
    check = Serial.read();
  
  switch(select_num){
  case ACCEL_NUM:
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      delay(75);
    }
  
    while(Wire.available()){
      _data.Data.accel.byte_data[i] = Wire.read();
      i++;
    }
    Serial.println(_data.Data.gyro.float_data.xG);
    break;
    
  case GYRO_NUM:
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      delay(70);
    }
  
    while(Wire.available()){
      _data.Data.gyro.byte_data[i] = Wire.read();
      i++;
    }
    Serial.println(_data.Data.gyro.float_data.xG);
    break;
 
  case GPS_NUM:
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*2);
      delay(70);
    }
  
    while(Wire.available()){
      _data.Data.gps.byte_data[i] = Wire.read();
      i++;
    }
    Serial.println(_data.Data.gps.gps_data.flat);
    break;
    
    case ALL_NUM:
      while(!Wire.available()){
        Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*8);
        delay(70);
      }
      
      while(Wire.available()){
        _data.byte_data[i] = Wire.read();
        i++;
      }
      Serial.println(_data.Data.gyro.float_data.xG);
      break;
      
    default :
      break;
  }  
  
}

float SerialMaster::get(char sensor, char axis)
{
  switch(sensor) {
    case GPS_NUM:
      switch(axis){
        case 'x':
          return(_data.Data.gps.gps_data.flat);
          break;
        case 'y':
          return(_data.Data.gps.gps_data.flon);
          break;
      }
     break;
     
     case ACCEL_NUM:
       switch(axis) {
         case 'x':
           return(_data.Data.accel.float_data.xA);
           break;
         case 'y':
           return(_data.Data.accel.float_data.yA);
           break;
         case 'z':
           return(_data.Data.accel.float_data.zA);
           break;
       }
       break;
       
     case GYRO_NUM:
       switch(axis) {
         case 'x':
           return(_data.Data.gyro.float_data.xG);
           break;
         case 'y':
           return(_data.Data.gyro.float_data.yG);
           break;
         case 'z':
           return(_data.Data.gyro.float_data.zG);
           break;
       }
       break;
  }
}
            
       
int SerialMaster::saveLog(char *str, float value)
{
  if(!SD.begin(SDPIN)) 
    return(-1);
    
  saveFile = SD.open("control_log.txt", FILE_WRITE);
  if(saveFile) {
    saveFile.print(str);
    saveFile.println(value);
    saveFile.close();
  }
  
  else
   return(-1);
}

int SerialMaster::saveLog(char *str)
{
  if(!SD.begin(SDPIN))
    return(-1);
    
  saveFile = SD.open("control_log.txt", FILE_WRITE);
  if(saveFile) {
      saveFile.println(str);
      saveFile.close();
  }
  
  else
    return(-1);
}
