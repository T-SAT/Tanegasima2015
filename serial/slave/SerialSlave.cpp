#include <Wire.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "SerialSlave.h"
#include "SensorStick_9DoF.h"

SerialSlave Slave;
sensorData _data;

SoftwareSerial ss(12, 11);
TinyGPS gpsSerial;

SerialSlave::SerialSlave()
{
  ss.begin(9600);
}

void SerialSlave::receive_data(ring_buffer *buf)
{
  char check;

  noInterrupts();
  check = Serial.read();
  interrupts();
  delay(100);
  Serial.print(RECEIVE);
  Wire.begin(SLAVE_DEVICE_NUM);

  if(check == GPS_NUM || check == ACCEL_NUM || check == GYRO_NUM || check == ALL_NUM){
    switch(check){
    case GPS_NUM:
      _data.Data.gps.gps_data.flat = recvGPS("lat");
      _data.Data.gps.gps_data.flon = recvGPS("lon");
      Wire.onRequest(send_GPS);
      Serial.print(START);
      break;

    case ACCEL_NUM:
      IMU.receiveAcc();
      _data.Data.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
      _data.Data.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');
      _data.Data.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');
      Wire.onRequest(send_Accel);
      Serial.print(START);
      break;

    case GYRO_NUM:
      IMU.receiveGyro();
      _data.Data.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');
      _data.Data.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
      _data.Data.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
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

float SerialSlave::recvGPS(char *select)
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
    return(select == "lat" ? flat : flon);
  }
}  





