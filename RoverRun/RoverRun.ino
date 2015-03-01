#include <Wire.h>
#include <SD.h>
#include <wiring_private.h>
#include "Run.h"
#include "SerialMaster.h"
#include "KalmanFilter.h"
#include <SoftwareSerial.h>

void setup()
{
  GEDE tmp1, tmp2, goal, current;
  ENU enu1, enu2;
  PolarCoordinates polar1, polar2;
  sensorData tmp;
  float distance=0;
  double dt;
  float angle;

  Serial.begin(9600);
  run.motorInit(6, 7, 5, 4);
  pinMode(10, OUTPUT);
  SD.begin(8);
  Wire.begin();

  /*  
   goal.LAT = 35.515659;
   goal.LON = 134.171860;
   run.setCoordinates("goal", goal.LAT, goal.LON);
   Master.request_data(GPS_NUM);
   tmp1.LAT = Master.get(GPS_NUM,'x');
   tmp1.LON = Master.get(GPS_NUM,'y');
   run.setCoordinates("landiing", tmp1.LAT, tmp1.LON);
   Master.saveLog(saveFile, "landing lat :", tmp1.LAT, millis());
   Master.saveLog(saveFile, "landing lon :", tmp1.LON, millis());
   
  /*
   do {
   Master.request_data(ALL_NUM);
   current.LAT = Master.get(GPS_NUM,'x');
   current.LON = Master.get(GPS_NUM,'y');
   tmp.Data.accel.float_data.xA = Master.get(ACCEL_NUM,'x');
   tmp.Data.accel.float_data.yA = Master.get(ACCEL_NUM,'y');
   tmp.Data.accel.float_data.zA = Master.get(ACCEL_NUM,'z');
   tmp.Data.gyro.float_data.zG = Master.get(GYRO_NUM,'z');
   current.LAT = Master.get(GPS_NUM,'x');
   current.LON = Master.get(GPS_NUM,'y');
   dt = Kalman.getDt();
   distance += (sqrt(pow(tmp.Data.accel.float_data.xA, 2) + pow(tmp.Data.accel.float_data.yA, 2))*9.8*pow(dt, 2))/2.0;
   run.steer(tmp.Data.gyro.float_data.zG, 0.0);
   }
   while(distance < 5.0);
   run.motor_control(0.0, 0.0);
  /*
   Master.request_data(GPS_NUM);
   tmp2.LAT = Master.get(GPS_NUM,'x');
   tmp2.LON = Master.get(GPS_NUM,'y');
   enu1 = run.GEDE2ENU(tmp1, tmp2, HEIGHT);
   enu2 = run.GEDE2ENU(tmp2, goal, HEIGHT);
   angle = run.get_angle(enu1.E, enu1.N, enu2.E, enu2.N);
   
   run.setCoordinates("origin", tmp2.LAT, tmp2.LON);
   Master.saveLog(saveFile, "origin lat :", tmp2.LAT, millis());
   Master.saveLog(saveFile, "origin lon :", tmp2.LON, millis());
   run.setENU("origin", enu1);
   run.setENU("goal", enu2);
   run.setPolarCoordinates("goal", sqrt(pow(enu2.E, 2) + pow(enu2.N, 2)), angle);
   run.setPolarCoordinates("rover", 0.0, 0.0);
   */
}

void loop()
{
  GEDE gede;
  sensorData data;
  float accXval, accYval, accZval;
  float gyroXval, gyroYval, gyroZval;
  float distanceOFgoal2current;

  Master.request_data(ALL_NUM);
  data.Data.accel.float_data.xA = Master.get(ACCEL_NUM,'x');
  data.Data.accel.float_data.yA = Master.get(ACCEL_NUM,'y');
  data.Data.accel.float_data.zA = Master.get(ACCEL_NUM,'z');
  data.Data.gyro.float_data.xG = Master.get(GYRO_NUM,'x');
  data.Data.gyro.float_data.yG = Master.get(GYRO_NUM,'y');
  data.Data.gyro.float_data.zG = Master.get(GYRO_NUM,'z');
  data.Data.gps.gps_data.flat = Master.get(GPS_NUM,'x');
  data.Data.gps.gps_data.flon = Master.get(GPS_NUM,'y');

  Master.saveData(data);
  double dt = run.getDt();

  /*
   ///////////////値更新//////////////////////
   run.update_PolarCoordinates((sqrt(pow(accXval,2) + pow(accYval,2))*9.8*pow(dt, 2))/2, gyroZval*dt);
   run.update_targetValue(gyroZval, dt);
   ///////////////////////////////////////////
   
   //////////////ゴール判定///////////////////
   distanceOFgoal2current = run.distanceOFgoal2current(gede);
   if(distanceOFgoal2current < 1.0)
   while(1);
   ///////////////////////////////////////////
   
   /////////////ずれ修正//////////////////////
   run.improveCurrentCoordinates(gede);
   //////////////////////////////////////////
   
   /////////////ステアリング制御/////////////
   run.steer(gyroZval, run.get_targetValue());
   //////////////////////////////////////////
   */
}














