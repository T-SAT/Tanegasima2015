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
    delay(100);
    check = Serial.read();
  }
  
  switch(select_num){
  case ACCEL_NUM:
    while(!Wire.available())
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
  
    while(Wire.available()){
      _data.accel.byte_data[i] = Wire.read();
      i++;
    }
    break;
    
  case GYRO_NUM:
    while(!Wire.available())
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      
      while(Wire.available()){
        _data.gyro.byte_data[i] = Wire.read();
        i++;
      }
      break;
     
  case GPS_NUM:
    while(!Wire.available())
      Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
      
      while(Wire.available()){
        _data.gps.byte_data[i] = Wire.read();
        i++;
      }
      break;
  }
  
  Serial.print("test1.f_data.f_data1 = ");
  Serial.println(_data.accel.float_data.xA);
}

