#include <Wire.h>
#include "SerialSlave.h"
#include "SensorStick_9DoF.h"

SerialSlave Slave;

sensorData _data;

void SerialSlave::select_func(int byte_size)
{  
  int select_num=2;
  
  while(Wire.available()){
    select_num = Wire.read();
  }
  
  switch (select_num) {
    case GPS_NUM:
      Wire.onRequest(send_GPS);
      break;
      
    case 2:
      interrupts();
      Wire.endTransmission();
      Wire.begin(8);
      Wire.onRequest(send_Accel);
      break;
      
    case GYRO_NUM:
      Wire.onRequest(send_Gyro);
      break;
      
    default:
      Wire.write(ERR_NUM);
      break;
  }
}

void SerialSlave::change_job(ring_buffer *buf)
{
  byte select = 1;
  
  noInterrupts();
  select = Serial.read() - '0';
  interrupts();
  Serial.print(RECIEVE);
  Wire.endTransmission(false);
  Wire.begin(8);
  Wire.onReceive(select_func);
}

void SerialSlave::send_GPS(void)
{
  Wire.write(_data.gps.byte_data, sizeof(_data.gps.float_data));
  Wire.endTransmission(false);
  IMU.sensorInit();
}

void SerialSlave::send_Accel(void)
{ 
  Serial.print("check");
  delay(10000);
  _data.accel.float_data.xA = 13.32;
  _data.accel.float_data.yA = 12.23;
  _data.accel.float_data.zA = 23.33;
  Wire.write(_data.accel.byte_data, sizeof(_data.accel.byte_data));
  Wire.endTransmission(false);
  IMU.sensorInit();
  noInterrupts();
}

void SerialSlave::send_Gyro(void)
{
  Wire.write(_data.gyro.byte_data, sizeof(_data.gyro.float_data));
  Wire.endTransmission();
  IMU.sensorInit();
}

void SerialSlave::setData_GPS(float flat, float flon, unsigned long int age)
{
  _data.gps.float_data.flat = flat;
  _data.gps.float_data.flon = flon;
}

void SerialSlave::setData_Accel(int x, int y, int z)
{
  _data.accel.float_data.xA = x;
  _data.accel.float_data.yA = y;
  _data.accel.float_data.zA = z;
}

void SerialSlave::setData_Gyro(int x, int y, int z)
{
  _data.gyro.float_data.xG = x;
  _data.gyro.float_data.yG = y;
  _data.gyro.float_data.zG = z;
}

int SerialSlave::saveSD(sensorData data){
  return(0);
}


