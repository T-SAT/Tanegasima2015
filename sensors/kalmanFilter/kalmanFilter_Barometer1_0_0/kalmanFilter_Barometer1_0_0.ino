#include <Wire.h>
#include <LPS331.h>
#include "SensorStick_9DoF.h"

LPS331 ps;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
    if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  sensorInit();
  ps.enableDefault();
}

void loop()
{
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
  
  IMU.receiveAcc();
  IMU.receiveGyro();
  
  double gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  double accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  double accZval = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  
  Serial.print("gyroX: ");
  Serial.print(gyroX);
  Serial.print("accYval\ta:");
  Serial.print(accYval);
  Serial.print("accZval\tt:");
  Serial.print(accZval);
  Serial.println(" [deg/m]");

  delay(100);
}

/*
double kalmanFilter_Barometer(double accel, double advanced, double init_advanced, double dt)
{
  static double x[2] = {init_advanced, 0};
  static double P[2][2] = { {0, 0}, {0, 0} };
  static double K[2];
  const double Q[2][2] = { {0.01, 0}, {0, 0.003} };
  const double R = 1;
  
  x[0] += dt * (u - x[1]);

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q[1][1] * dt;
    
  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);

  x[0] += K[0] * (y - x[0]);
  x[1] += K[1] * (y - x[0]);

  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
    
  return x[0];
}
*/

double getDt(void)
{
  static long lastTime=0;
  
  long nowTime = micros();
  double time = (double)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;
  
  return( time );  
}

void sensorInit(void){  
  IMU.begin();   
  delay(1000);
  
  double accZero[3] = { 0 }; // accX, accY, accZ
  double gyrZero[4] = { 0 }; // gyroX, gyroY, gyroZ,gyroTemp
  double magZero[3] = { 0 }; // magX, magY, magZ
  
  for (int i = 0; i < 100; i++) {
    IMU.receiveAll();
    accZero[0] += IMU.get(ACC,'x');
    accZero[1] += IMU.get(ACC,'y');
    accZero[2] += IMU.get(ACC,'z');
    gyrZero[0] += IMU.get(GYR,'x');
    gyrZero[1] += IMU.get(GYR,'y');
    gyrZero[2] += IMU.get(GYR,'z');
    delay(10);
  }
   accZero[0] /= 100;
   accZero[1] /= 100;
   accZero[2] /= 100;
   gyrZero[0] /= 100;
   gyrZero[1] /= 100;
   gyrZero[2] /= 100;
  
  accZero[2] -= 1;  //重力加速度の除去
  IMU.setZero(accZero,gyrZero,magZero);
}
