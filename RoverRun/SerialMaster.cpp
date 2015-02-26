#include <Wire.h>
#include "SerialMaster.h"

SerialMaster Master;
  
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
  unsigned long int time;
  
  time = millis();
  while(Serial.read() != ENABLE){
    if(1000 < millis() - time)
      break;
  }
  
  time = millis();
  while(check != RECEIVE){
    Serial.print(select_num);
    delay(60);
    check = Serial.read();
    if(1000 < millis() - time)
      break; 
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
      delay(60);
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
      delay(70);
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
      delay(70);
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
        Wire.requestFrom(SLAVE_DEVICE_NUM, (sizeof(float)*11 + sizeof(unsigned long int)));
        delay(75);
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
