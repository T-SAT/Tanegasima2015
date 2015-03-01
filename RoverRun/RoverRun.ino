#include <Wire.h>
#include <SD.h>
#include <wiring_private.h>
#include "Run.h"
#include "SerialMaster.h"
#include "KalmanFilter.h"
#include <SoftwareSerial.h>

GEDE lastGEDE;

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


  goal.LAT = 35.515659;
  goal.LON = 134.171860;
  run.GOAL.LAT = goal.LAT;
  run.GOAL.LON = goal.LON;
  Master.request_data(GPS_NUM);
  tmp1.LAT = Master.get(GPS_NUM,'x');
  tmp1.LON = Master.get(GPS_NUM,'y');
  run.LANDING.LAT = tmp1.LAT;
  run.LANDING.LON = tmp1.LON;
  Master.saveLog("landing lat :", tmp1.LAT);
  Master.saveLog("landing lon :", tmp1.LON);

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
  } while(distance < 5.0);
  */
  run.motor_control(0.0, 0.0);
  Master.request_data(GPS_NUM);
  tmp2.LAT = Master.get(GPS_NUM,'x');
  tmp2.LON = Master.get(GPS_NUM,'y');
  enu1 = run.GEDE2ENU(tmp1, tmp2, HEIGHT);
  enu2 = run.GEDE2ENU(tmp2, goal, HEIGHT);
  angle = run.get_angle(enu1.E, enu1.N, enu2.E, enu2.N);

  run.ORIGIN.LAT = tmp2.LAT; 
  run.ORIGIN.LON = tmp2.LON;
  Master.saveLog("origin lat :", tmp2.LAT);
  Master.saveLog("origin lon :", tmp2.LON);
  run.originENU.E = enu1.E;
  run.originENU.N = enu1.N;
  run.goalENU.E = enu2.E;
  run.goalENU.N = enu2.N;
  run.goal.distance = pow(enu2.E, 2) + pow(enu2.N, 2);
  run.goal.angle = angle;
  run.rover.distance = 0.0; 
  run.rover.angle = 0.0;
  lastGEDE.LAT = tmp2.LAT;
  lastGEDE.LON = tmp2.LON;
}

void loop()
{
  GEDE currentGEDE;
  ENU tmp;
  sensorData data;
  float accXval, accYval, accZval, AccVal;
  float gyroXval, gyroYval, gyroZval;
  static double distance=0.0;
  static double last_distance = 0.0;
  double current_distance;
  float distanceOFgoal2current;

  Master.request_data(ALL_NUM);
  
  data.Data.accel.float_data.xA = Master.get(ACCEL_NUM,'x');
  data.Data.accel.float_data.yA = accYval = Master.get(ACCEL_NUM,'y');
  //data.Data.accel.float_data.zA = Master.get(ACCEL_NUM,'z');
  //data.Data.gyro.float_data.xG = Master.get(GYRO_NUM,'x');
  //data.Data.gyro.float_data.yG = Master.get(GYRO_NUM,'y');
  data.Data.gyro.float_data.zG = Master.get(GYRO_NUM,'z');
  data.Data.gps.gps_data.flat = currentGEDE.LAT = Master.get(GPS_NUM,'x');
  data.Data.gps.gps_data.flon = currentGEDE.LON = Master.get(GPS_NUM,'y');
  //Master.saveData(data);
  
  Master.saveLog("AX",data.Data.accel.float_data.xA);
  Master.saveLog("AY",data.Data.accel.float_data.yA);
  Master.saveLog("ZG",data.Data.gyro.float_data.zG);
  Master.saveLog("LAT",data.Data.gps.gps_data.flat);
  Master.saveLog("LON",data.Data.gps.gps_data.flon);
  accYval = -accYval;
  AccVal = sqrt(pow(accYval, 2) + pow(accXval, 2)) * 9.8;
  tmp = run.GEDE2ENU(lastGEDE, currentGEDE, HEIGHT);
  distance += sqrt(pow(tmp.E,2) + pow(tmp.N,2));
  Master.saveLog("distance from origin", distance);
  double dt = run.getDt();

  ///////////////値更新//////////////////////
  current_distance = Kalman.kalmanFilter_Distance(AccVal, distance, dt);
  run.update_PolarCoordinates(current_distance - last_distance, gyroZval*dt);
  run.update_targetValue(gyroZval, dt);
  lastGEDE.LAT = currentGEDE.LAT;
  lastGEDE.LON = currentGEDE.LON;
  last_distance = current_distance;
  ///////////////////////////////////////////

  //////////////ゴール判定///////////////////
  distanceOFgoal2current = run.distanceOFgoal2current(currentGEDE);
  if(distanceOFgoal2current < 1.0){
    Master.saveLog("goal coordinate lat",currentGEDE.LAT);
    Master.saveLog("goal coordinate lon",currentGEDE.LON);
    while(1);
  }
  ///////////////////////////////////////////

  /////////////ずれ修正//////////////////////
  run.improveCurrentCoordinates(currentGEDE);
  //////////////////////////////////////////

  /////////////ステアリング制御/////////////
  run.steer(data.Data.gyro.float_data.zG, run.get_targetValue());
  //////////////////////////////////////////

}
















