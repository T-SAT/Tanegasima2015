#include <LPS331.h>
#include "SensorStick_9DoF.h"
#include "KalmanFilter.h"
#include <Wire.h>

LPS331 ps;
KalmanFilter KalmanAltitude;
KalmanFilter KalmanAccelX;
KalmanFilter KalmanAccelY;
KalmanFilter KalmanAccelZ;
KalmanFilter KalmanGyroX;
KalmanFilter KalmanGyroY;
KalmanFilter KalmanGyroZ;
KalmanFilter KalmanGPS;

void setup() {
  float pressure, tempreture;
  
  Serial.begin(9600); 
  IMU.sensorInit();
  ps.init();
  ps.enableDefault();
  pressure = ps.readPressureMillibars();
  tempreture = ps.readTemperatureC();
  ps.set_initValue(pressure, tempreture);
  KalmanAltitude.setState(0.0);
  KalmanAccelX.setState(0.0);
  KalmanAccelY.setState(0.0);
  KalmanAccelZ.setState(1.0);
  KalmanGyroX.setState(0.0);
  KalmanGyroY.setState(0.0);
  KalmanGyroZ.setState(0.0);
  KalmanGPS.setState(0.0);
}

void loop() { 
  //⊿tの測定 bh
  double dt = Kalman.getDt();
  unsigned long int time;
  double gyroX; 
  double accYval;
  double accZval;
  double accXval;
  //各手法による角度の導出
  static double angle_gyro = 180;
  double angle_acc = 180;
  double angle_kalman, high_kalman;
  double add_accZval;
  double errorValue=0;
  float pressure;
  double altitude;
  static double last_altitude = 0.0;
  
  time = millis();
  pressure = ps.readPressureMillibars();
  altitude = ps.pressureToAltitudeMeters(pressure);
  IMU.receiveAcc();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  accZval = IMU.get(ACC,'z');//IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  accXval = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');

  KalmanAltitude.correct(altitude);
  KalmanAccelX.correct(accXval);
  KalmanAccelY.correct(accYval);
  KalmanAccelZ.correct(accZval);
  KalmanGyroX.correct(gyroX);
  
  altitude = KalmanAltitude.getState();
  accXval = KalmanAccelX.getState();
  accYval = KalmanAccelY.getState();
  accZval = KalmanAccelZ.getState();
  gyroX = KalmanGyroX.getState();
  
  angle_gyro  += gyroX*dt;                            //ジャイロセンサ(角速度の積分)
  angle_acc   += atan2(accYval, accZval)*RAD_TO_DEG;  //加速度センサ(重力加速度の向きから)
  angle_kalman = Kalman.kalmanFilter_9DOF(gyroX,angle_acc,dt);      //カルマンフィルタ(加速度センサの角度とジャイロの角速度)
  add_accZval = accXval*sin(angle_kalman-180.0) + accZval*cos(angle_kalman-180.0);
  
  if(altitude - last_altitude < LPS331_ERRORVALUE && -LPS331_ERRORVALUE < altitude - last_altitude)
    add_accZval = 0.0;
    

  high_kalman = Kalman.kalmanFilter_Barometer(add_accZval,altitude, dt);
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
  Serial.print(altitude); 
  Serial.print(",");  
  Serial.print(high_kalman);
  Serial.print(",");    Serial.println(time);
}


