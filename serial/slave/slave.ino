//com12: slave
#include <Wire.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <LPS331.h>
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"
#include "KalmanFilter.h"
#include <MsTimer2.h>

LPS331 ps;
SoftwareSerial ss(12, 11);
TinyGPS gpsSerial;

KalmanFilter Kalman9DOFX;
KalmanFilter Kalman9DOFY;

/*
KalmanFilter KalmanAltitude;
KalmanFilter KalmanAccelX;
KalmanFilter KalmanAccelY;
KalmanFilter KalmanAccelZ;
KalmanFilter KalmanGyroX;
KalmanFilter KalmanGyroY;
KalmanFilter KalmanGyroZ;
KalmanFilter KalmanGPSFlat;
KalmanFilter KalmanGPSFlon;
*/

void setup() {
  float pressure, tempreture;
  float flat,flon;
  
  Serial.begin(9600); 
  ss.begin(9600);
  sensorInit();
  pinMode(10,OUTPUT);
  
  /*
  pressure = ps.readPressureMillibars();
  tempreture = ps.readTemperatureC();
  ps.set_initValue(pressure, tempreture);
  
  KalmanAltitude.setState(0.0);
  */
  
  /*
  flat = recvGPS('x');
  flon = recvGPS('y');
  KalmanAccelX.setState(0.0);
  KalmanAccelY.setState(0.0);
  KalmanAccelZ.setState(1.0);
  KalmanGyroX.setState(0.0);
  KalmanGyroY.setState(0.0);
  KalmanGyroZ.setState(0.0);
  KalmanGPSFlat.setState(flat);
  KalmanGPSFlon.setState(flon);
  */
  Serial.print(ENABLE);
  Serial.setintr(Slave.receive_data);
}

void loop() { 
  //⊿tの測定 bh
  sensorData tmp;
  unsigned long int time;
  int check_sd;
  
  /*
  IMU.receiveAcc();
  IMU.receiveGyro();
  time = millis();
  tmp.Data.gyro.float_data.xG = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  tmp.Data.gyro.float_data.yG = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
  tmp.Data.gyro.float_data.zG = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
  tmp.Data.accel.float_data.xA = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  tmp.Data.accel.float_data.yA = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  tmp.Data.accel.float_data.zA = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  tmp.Data.gps.gps_data.flat = recvGPS('x');
  tmp.Data.gps.gps_data.flon = recvGPS('y');
 
 /*
  KalmanAccelX.correct(tmp.Data.accel.float_data.xA);
  KalmanAccelY.correct(tmp.Data.accel.float_data.yA);
  KalmanAccelZ.correct(tmp.Data.accel.float_data.zA);
  KalmanGyroX.correct(tmp.Data.gyro.float_data.xG);
  KalmanGyroY.correct(tmp.Data.gyro.float_data.yG);
  KalmanGyroZ.correct(tmp.Data.gyro.float_data.zG);
  KalmanGPSFlat.correct(tmp.Data.gps.gps_data.flat);
  KalmanGPSFlon.correct(tmp.Data.gps.gps_data.flat);
  tmp.Data.accel.float_data.xA = KalmanAccelX.getState();
  tmp.Data.accel.float_data.yA = KalmanAccelY.getState();
  tmp.Data.accel.float_data.zA = KalmanAccelZ.getState();
  tmp.Data.gyro.float_data.xG  = KalmanGyroX.getState();
  tmp.Data.gyro.float_data.yG  = KalmanGyroY.getState();
  tmp.Data.gyro.float_data.zG  = KalmanGyroZ.getState();
  tmp.Data.gps.gps_data.flat   = KalmanGPSFlat.getState();
  tmp.Data.gps.gps_data.flon   = KalmanGPSFlon.getState();
  */
  /*
  Slave.setData_Accel(tmp.Data.accel.float_data.xA, tmp.Data.accel.float_data.yA, tmp.Data.accel.float_data.zA);
  Slave.setData_Gyro(tmp.Data.gyro.float_data.xG, tmp.Data.gyro.float_data.yG, tmp.Data.gyro.float_data.zG);
  Slave.setData_GPS(tmp.Data.gps.gps_data.flat, tmp.Data.gps.gps_data.flon);
  */
  
  /*
  do {
    check_sd = Slave.saveSD(tmp, time);
  }while(check_sd == -1);
  */
  
  //Serial.print(ENABLE);
  
  delay(100);
}

void sensorInit()
{
  IMU.sensorInit();
  ps.init();
  ps.enableDefault();
}

void cut_parachute(void)
{
  pinMode(PARA_PIN, OUTPUT);
  digitalWrite(PARA_PIN, HIGH);
  delay(2000);
  digitalWrite(PARA_PIN, LOW);
}

float recvGPS(char select)
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed; 

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
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    return(select == 'x' ? flat : flon);
  }
}  


