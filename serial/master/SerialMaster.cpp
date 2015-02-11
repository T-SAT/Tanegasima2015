#include <Wire.h>
#include "serial_master.h"

SerialMaster Master;
  
void SerialMaster::write_numbers(int numbers, int dev_number){
	Wire.beginTransmission(dev_number);
	Wire.write(numbers);
	Wire.endTransmission();
	delay(100);
}

void SerialMaster::request_data(int select_num)
{
  int i=0;
  
  switch(select_num){
  case ACCEL_NUM:  
    while(!Wire.available()){
      write_numbers(select_num, SLAVE_DEVICE_NUM);
      Wire.begin();
      Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
    }

    while(!Wire.available()) 
      Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
  
    while(Wire.available()){
      _data.accel.byte_data[i] = Wire.read();
      i++;
    }
    break; 

  case GYRO_NUM:
    while(!Wire.available()){
      write_numbers(select_num, SLAVE_DEVICE_NUM);
      Wire.begin();
      Wire.requestFrom(SLAVE_DEVICE_NUM, GYRO_REQUEST_BYTE);
    }

    while(!Wire.available()) 
      Wire.requestFrom(SLAVE_DEVICE_NUM, GYRO_REQUEST_BYTE);
  
    while(Wire.available()){
      _data.accel.byte_data[i] = Wire.read();
      i++;
    }  
   break;
   
  case GPS_NUM:
    while(!Wire.available()){
      write_numbers(select_num, SLAVE_DEVICE_NUM);
      Wire.begin();
      Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
    }

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
