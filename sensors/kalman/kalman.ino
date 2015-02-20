#include <LPS331.h>
#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include <Wire.h>

LPS331 ps;

void setup() {
  float pressure, tempreture;
  
  Serial.begin(9600); 
  IMU.sensorInit();
  ps.init();
  ps.enableDefault();
  pressure = ps.readPressureMillibars();
  tempreture = ps.readTemperatureC();
  ps.set_initValue(pressure, tempreture);
}

void loop() { 
  //⊿tの測定 bh
  double dt = Kalman.getDt();
  unsigned long int time;
  
  //センサデータの取得
  time = millis();
  IMU.receiveAcc();
  IMU.receiveGyro();
  double gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  double accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  double accZval = IMU.get(ACC,'z');//IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  double accXval = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  //各手法による角度の導出
  static double angle_gyro = 180;
  double angle_acc = 180;
  double angle_kalman, high_kalman;
  double add_accZval;
  float pressure = ps.readPressureMillibars();
  double altitude = ps.pressureToAltitudeMeters(pressure);
  static double last_altitude = 0.0;
  
  angle_gyro  += gyroX*dt;                            //ジャイロセンサ(角速度の積分)
  angle_acc   += atan2(accYval, accZval)*RAD_TO_DEG;  //加速度センサ(重力加速度の向きから)
  angle_kalman = Kalman.kalmanFilter_9DOF(gyroX,angle_acc,dt);      //カルマンフィルタ(加速度センサの角度とジャイロの角速度)
  add_accZval = accXval*sin(angle_kalman-180.0) + accZval*cos(angle_kalman-180.0);
  
  /*
  if(altitude - last_altitude < LPS331_ERRORVALUE && -LPS331_ERRORVALUE < altitude - last_altitude)
    add_accZval = 0.0;
  */
    
  high_kalman = Kalman.kalmanFilter_Barometer(add_accZval, altitude, dt);
  //シリアルで出力
  
  //Serial.print("altitude = ");  Serial.println(altitude);
  //Serial.print("accZval = ");  Serial.println(accZval);
  /*
  Serial.print(angle_gyro);   Serial.print('\t');
  Serial.print(angle_acc);    Serial.print('\t');
  Serial.print(angle_kalman); Serial.print('\t');
  Serial.print('\n');
  */
  last_altitude = altitude;
  //Serial.print(altitude); Serial.print(",");  
  Serial.println(high_kalman);
  //Serial.print(",");    Serial.println(time);
}


