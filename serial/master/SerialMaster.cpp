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
  
  while(check != RECEIVE){
    Serial.print(select_num);
    delay(60);
    check = Serial.read();
  }
  Serial.println("check");
  while(check != START)
    check = Serial.read();
  
  switch(select_num){
  case ACCEL_NUM:
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      delay(75);
    }
  
    while(Wire.available()){
      _data.accel.byte_data[i] = Wire.read();
      i++;
    }
    Serial.println(_data.gyro.float_data.xG);
    break;
    
  case GYRO_NUM:
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      delay(70);
    }
  
    while(Wire.available()){
      _data.gyro.byte_data[i] = Wire.read();
      i++;
    }
    Serial.println(_data.gyro.float_data.xG);
    break;
 
  case GPS_NUM:
    while(!Wire.available()){
      Wire.requestFrom(SLAVE_DEVICE_NUM, (sizeof(float)*2+sizeof(unsigned long int)));
      delay(70);
    }
  
    while(Wire.available()){
      _data.gps.byte_data[i] = Wire.read();
      i++;
    }
    Serial.println(_data.gps.gps_data.flat);
    break;
    
    case ALL_NUM:
      while(!Wire.available()){
        Wire.requestFrom(SLAVE_DEVICE_NUM, (sizeof(float)*11 + sizeof(unsigned long int)));
        delay(70);
      }
      
      while(Wire.available()){
        _data.byte_data[i] = Wire.read();
        i++;
      }
      Serial.println(_data.gyro.float_data.xG);
      break;
      
    default :
      break;
  }  
  
}
