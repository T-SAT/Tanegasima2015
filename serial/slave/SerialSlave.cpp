#include <Wire.h>
#include "SerialSlave.h"
#include "SensorStick_9DoF.h"

SerialSlave Slave;

sensorData _data;

void SerialSlave::select_func(void)
{
  int select_num;

  while(Wire.available()){
     select_num = Wire.read();
  }
  
  switch (select_num) {
    case GPS_NUM:
      Wire.onRequest(send_GPS);
      break;
      
    case ACCEL_NUM:
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
  byte check = 1;
  noInterrupts();
  check = Serial.read() - '0';
  
  if(check == START){
    interrupts();
    Serial.print(RECIEVE);
    Wire.endTransmission();
    Wire.begin(SLAVE_DEVICE_NUM);
    Wire.onRequest(select_func);
  }
  
}

void SerialSlave::send_GPS(void)
{
  Wire.write(_data.gps.byte_data, sizeof(_data.gps.float_data));
  noInterrupts();
  Wire.endTransmission();
  IMU.sensorInit();
}

void SerialSlave::send_Accel(void)
{
  Wire.write(_data.accel.byte_data, sizeof(_data.accel.float_data));
  noInterrupts();
  Wire.endTransmission();
  IMU.sensorInit();
}

void SerialSlave::send_Gyro(void)
{
  Wire.write(_data.gyro.byte_data, sizeof(_data.gyro.float_data));
  noInterrupts();
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


