#include <Wire.h>
#include "SerialMaster.h"

SerialMaster Master;
  
void SerialMaster::write_numbers(int numbers, int dev_number){
	Wire.beginTransmission(dev_number);
	Wire.write(numbers);
	Wire.endTransmission();
	delay(100);
}

void SerialMaster::request_data(byte select_num)
{
  int i=0;
  byte check=0;
  
  do {
    Serial.print(select_num);
    delay(100);
    if(Serial.available()){
      check = Serial.read() - '0';
    }
  }while(check != RECIEVE);
  
  switch(select_num){
    case ACCEL_NUM: 
      while(!Wire.available()){
        Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
      }
     while(Wire.available()){
       _data.accel.byte_data[i] = Wire.read();
       i++;
     }
     Serial.println(_data.accel.float_data.xA);
     break; 

    case GYRO_NUM:
      Serial.print(GYRO_NUM);
      while(!Wire.available()) 
        Wire.requestFrom(SLAVE_DEVICE_NUM, GYRO_REQUEST_BYTE);
  
       while(Wire.available()){
         _data.accel.byte_data[i] = Wire.read();
         i++;
       }  
      break;
   
   case GPS_NUM:
      Serial.print(GPS_NUM);
      while(!Wire.available()) 
        Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
  
      while(Wire.available()){
        _data.accel.byte_data[i] = Wire.read();
        i++;
      }
     break;
   
    default :
      break;
  }
}
