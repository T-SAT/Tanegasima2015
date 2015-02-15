#include <Wire.h>
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
      break;
  
    case ACCEL_NUM:
      Wire.onRequest(send_Accel);
      Serial.print(START);
      break;
    
    case GYRO_NUM:
      Wire.onRequest(send_Gyro);
      break;
    
    default :
      break; 
    }
  }
  
}

void SerialSlave::send_GPS(void)
{
  Wire.write(_data.gps.byte_data, sizeof(_data.gps.byte_data));
  IMU.sensorInit();
}

void SerialSlave::send_Accel(void)
{ 
  Wire.write(_data.accel.byte_data, sizeof(_data.accel.byte_data));
  Wire.begin();
  noInterrupts();
}

void SerialSlave::send_Gyro(void)
{
  Wire.write(_data.gyro.byte_data, sizeof(_data.gyro.float_data));
  Wire.begin();
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

int SerialSlave::saveSD(sensorData data){
  return(0);
}


