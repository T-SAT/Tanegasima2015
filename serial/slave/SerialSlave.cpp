#include <Wire.h>
#include <SD.h>
#include <TinyGPS.h>
#include "SerialSlave.h"
#include "SensorStick_9DoF.h"
#include <SoftwareSerial.h>

SerialSlave Slave;
sensorData _data;
SoftwareSerial ss(12, 11);
SoftwareSerial ssRadio(2, 3);
TinyGPS gpsSerial;

SerialSlave::SerialSlave()
{
  ss.begin(9600);
  //ssRadio.begin(9600);
}

void SerialSlave::receive_data(ring_buffer *buf)
{
  char check;

  noInterrupts();
  check = Serial.read();
  interrupts();
  Serial.print(RECEIVE);

  if(check == GPS_NUM || check == ACCEL_NUM || check == GYRO_NUM || check == ALL_NUM){
    switch(check){
    case GPS_NUM:
      _data.Data.gps.gps_data.flat = recvGPS('x');
      _data.Data.gps.gps_data.flon = recvGPS('y');
      saveRadio_data(millis());
      Wire.onRequest(send_GPS);
      Wire.begin(SLAVE_DEVICE_NUM);
      Serial.print(START);
      break;

    case ACCEL_NUM:
      IMU.receiveAcc();
      _data.Data.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
      _data.Data.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');
      _data.Data.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');
      saveRadio_data(millis());
      Wire.begin(SLAVE_DEVICE_NUM);
      Wire.onRequest(send_Accel);
      Serial.print(START);
      break;

    case GYRO_NUM:
      IMU.receiveGyro();
      _data.Data.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');
      _data.Data.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
      _data.Data.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
      saveRadio_data(millis());
      Wire.begin(SLAVE_DEVICE_NUM);
      Wire.onRequest(send_Gyro);
      Serial.print(START);
      break;

    case ALL_NUM:
      IMU.receiveGyro();
      IMU.receiveAcc();
      _data.Data.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');
      _data.Data.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
      _data.Data.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
      _data.Data.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
      _data.Data.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');
      _data.Data.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');
      _data.Data.gps.gps_data.flat = recvGPS('x');
      _data.Data.gps.gps_data.flon = recvGPS('y');
      saveRadio_data(millis());
      Wire.begin(SLAVE_DEVICE_NUM);
      Wire.onRequest(send_All);
      Serial.print(START);
      break;

    default :
      break; 
    }
  }

}

void SerialSlave::send_GPS(void)
{
  Wire.write(_data.Data.gps.byte_data, sizeof(_data.Data.gps.byte_data));
  Serial.print(ENABLE);
  Wire.begin();
}

void SerialSlave::send_Accel(void)
{ 
  Wire.write(_data.Data.accel.byte_data, sizeof(_data.Data.accel.byte_data));
  Serial.print(ENABLE);  
  Wire.begin();
}

void SerialSlave::send_Gyro(void)
{
  Wire.write(_data.Data.gyro.byte_data, sizeof(_data.Data.gyro.byte_data));
  Serial.print(ENABLE);
  Wire.begin();
}

void SerialSlave::send_All(void)
{
  Wire.write(_data.byte_data, sizeof(_data.byte_data));
  Serial.print(ENABLE);
  Wire.begin();
}

void SerialSlave::setData_GPS(float flat, float flon)
{
  _data.Data.gps.gps_data.flat = flat;
  _data.Data.gps.gps_data.flon = flon;
}

void SerialSlave::setData_Accel(float x, float y, float z)
{
  _data.Data.accel.float_data.xA = x;
  _data.Data.accel.float_data.yA = y;
  _data.Data.accel.float_data.zA = z;
}

void SerialSlave::setData_Gyro(float x, float y, float z)
{
  _data.Data.gyro.float_data.xG = x;
  _data.Data.gyro.float_data.yG = y;
  _data.Data.gyro.float_data.zG = z;
}

float SerialSlave::recvGPS(char select)
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed; 

  ss.begin(9600);
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  { 
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gpsSerial.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gpsSerial.f_get_position(&flat, &flon, &age);
/*
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
*/
    return(select=='x' ? flat : flon);
  }
}  

int SerialSlave::saveRadio_data(unsigned long int time)
{
  ssRadio.print(_data.Data.accel.float_data.xA);  
  ssRadio.print(",");
  ssRadio.print(_data.Data.accel.float_data.yA);  
  ssRadio.print(",");
  ssRadio.print(_data.Data.accel.float_data.zA);  
  ssRadio.print(",");
  ssRadio.print(_data.Data.gyro.float_data.xG);  
  ssRadio.print(",");
  ssRadio.print(_data.Data.gyro.float_data.yG);  
  ssRadio.print(",");
  ssRadio.print(_data.Data.gyro.float_data.zG);  
  ssRadio.print(",");
  ssRadio.print(_data.Data.gps.gps_data.flat,6); 
  ssRadio.print(",");
  ssRadio.print(_data.Data.gps.gps_data.flon,6); 
  ssRadio.print(",");
  ssRadio.print(time);
  ssRadio.println();

  return(0);
}

int SerialSlave::saveRadio_begin(long speed)
{
  ssRadio.begin(speed);
  ssRadio.print("xA,yA,zA,xG,yG,zG,LAT,LON,pressure,altitude,tempreture,time");
  ssRadio.println();
}

int SerialSlave::saveRadio(float data, unsigned long int time)
{
  ssRadio.print(data, 6);
  ssRadio.print(",");
  ssRadio.print(time);
  ssRadio.println();
  
  return(0);
}

