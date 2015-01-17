#include <Wire.h>
#include "SensorStick_9DoF.h"

void setup(){
  Serial.begin(115200); 
  sensorInit();
}

void loop(){ 
  //⊿tの測定
  double dt = getDt();
  
  //センサデータの取得
  IMU.receiveAcc();
  IMU.receiveGyro();
  double gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  double accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  double accZval = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  
  //各手法による角度の導出
  static double angle_gyro = 180;
  double angle_acc = 180;
  double angle_kalman;
  angle_gyro  += gyroX*dt;                            //ジャイロセンサ(角速度の積分)
  angle_acc   += atan2(accYval, accZval)*RAD_TO_DEG;  //加速度センサ(重力加速度の向きから)
  angle_kalman = kalmanFiltering(gyroX,angle_acc,dt);      //カルマンフィルタ(加速度センサの角度とジャイロの角速度)
  
  //シリアルで出力
  Serial.print(angle_gyro);  Serial.print('\t');
  Serial.print(angle_acc);   Serial.print('\t');
  Serial.print(angle_kalman);Serial.print('\t');
  Serial.print('\n');
}


//カルマンフィルタ
double kalmanFiltering(double u,double y,double dt){
    static double x[2] = {180, 0};
    static double P[2][2] = { {0, 0}, {0, 0}};
    static double K[2];
    const  double Q[2][2] = { {0.01, 0}, {0, 0.003}};
    const  double R = 1;
  
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


//⊿tの測定
double getDt(){
  static long lastTime=0;
  
  long nowTime = micros();
  double time = (double)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;
  
  return( time );  
}


//センサ使用準備・ゼロ点取得
void sensorInit(){  
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
