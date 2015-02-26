#include "Run.h"
#include "SerialMaster.h"
#include <wiring_private.h>

Run run;

unsigned long int time00;
unsigned long int time01;
unsigned long int time02;
unsigned long int time10;
unsigned long int time11;
unsigned long int time12;
    
void Run::ADCInit(void)
{
  DDRC &= ~_BV(5);
}

void Run::motorInit(int LF, int LB, int RF, int RB)
{
  motorPinLF = LF;
  motorPinLB = LB;
  motorPinRF = RF;
  motorPinRB = RB;
  
  
  pinMode(motorPinLF, OUTPUT);
  pinMode(motorPinLB, OUTPUT);
  pinMode(motorPinRF, OUTPUT);
  pinMode(motorPinRB, OUTPUT);
}
 
void Run::rollInit(void)
{
  time00 = 0;
  time01 = 0;
  time02 = 0;
  time10 = 0;
  time11 = 0;
  time12 = 0;
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(0, get_rollInstrumentL, FALLING);
  attachInterrupt(1, get_rollInstrumentR, FALLING);
}

void Run::get_distance(void)
{
  
}

void Run::get_angle(void)
{
  
}

float Run::get_angle(float vec1X, float vec1Y, float vec2X, float vec2Y)
{
  float sin_v, cos_v;
  float seata;
  
  sin_v = (vec1X*vec2Y - vec2X*vec1Y)/(sqrt(vec1X*vec1X + vec1Y*vec1Y) * sqrt(vec2X*vec2X + vec2Y*vec2Y));
  cos_v = (vec1X*vec2X + vec1Y*vec2Y)/(sqrt(vec1X*vec1X + vec1Y*vec1Y) * sqrt(vec2X*vec2X + vec2Y*vec2Y));
 
  if(0<sin_v && 0<cos_v){
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y)*max(asin(sin_v), -asin(sin_v));
    return(seata);
  }

  else if(0<sin_v && cos_v<0){
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y)*max(acos(cos_v), -acos(cos_v));
    return(seata);
  }

  else if(sin_v<0 && cos_v<0){
    seata = acos(cos_v);
    seata = seata + PI/2;
    seata = 2*PI - seata;
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y)*max(seata, -seata);
    return(seata);
  }

  else if(sin_v<0 && 0<cos_v){
    seata = asin(sin_v);
    seata = crossProduct(vec1X, vec1Y, vec2X, vec2Y)*max(seata, -seata);
    return(seata);
  }
  
  else return(0);
}

int Run::crossProduct(float vec1X, float vec1Y, float vec2X, float vec2Y)
{
  float tmp;
  
  tmp = vec1X*vec2Y - vec2X*vec1Y;
  if(tmp < 0) return(1);
  else if(0 < tmp) return(-1);
  else return(0);
}
  
float Run::get_gyro(void)
{
  
}

unsigned long int Run::rollInstrumentL(void)
{
  return(time00);
}

unsigned long int Run::rollInstrumentR(void)
{
  return(time10);
}

void Run::get_rollInstrumentL(void)
{
  unsigned long int time;
  
  time = micros();;
  while(micros()-time < 500);
  
  time02 = millis();
  time00 = time02 - time01;
  time01 = time02;
}

void Run::get_rollInstrumentR(void)
{
  unsigned long int time;
  
  time = micros();
  while(micros()-time < 500);
  
  time12 = millis();
  time10 = time12 - time11;
  time11 = time12;
}
    
void Run::motor_control(int motorL, int motorR)
{
  
}

void Run::motor_controlVolt(float motorL, float motorR)
{
  float volt;
  int dutyL, dutyR;
  
  volt = batt_voltage();
  
  dutyL = (int)(motorL/volt) * 255;
  dutyR = (int)(motorR/volt) * 255;
  
  motor_control(dutyL, dutyR);
  
}

void Run::turn(float target_value)
{
  double p_gain=1.0; //Pゲイン
  double i_gain=0.001; //Iゲイン（多すぎると暴走します）
  double d_gain=0.5; //Dゲイン
  double control_value=0.0; //制御量（モータなどへの入力量）

//割り込みタイマー処理(H8ならITUなど。一定周期でループする。
  double current_value; //現在の出力値
  double last_value=0.0; //一つ前の出力値（要保存のためstatic）
  double error,d_error; //偏差、偏差の微小変化量
  double i_error=0.0; //偏差の総和（要保存のためstatic）
  double dt;
  
  do {
    dt = getDt();
    Master.request_data(GYRO_NUM);
    current_value = Master.get(GYRO_NUM, 'z') * dt; //センサ等から値を読み取る
    
    error = target_value - current_value; //偏差の計算
    d_error = current_value - last_value; //偏差微小変化量の計算

    control_value = p_gain*error + i_gain*i_error + d_gain*d_error;
   //制御量の計算
  
    if(target_value < 0)
      motor_controlVolt(-control_value, control_value); //モータへ値を代入

    else 
      motor_controlVolt(control_value, -control_value);  

    last_value=current_value; //一つ前の出力値を更新
    i_error+=error; //偏差の総和を更新
  } while(0 < error);

}

void Run::steer(void)
{
}

float Run::forward(float target_value)
{
  const double p_gain=1.0; //Pゲイン
  const double i_gain=0.001; //Iゲイン（多すぎると暴走します）
  const double d_gain=0.5; //Dゲイン
  double control_valueL=0.0; //制御量（モータなどへの入力量）
  double control_valueR=0.0;
//割り込みタイマー処理(H8ならITUなど。一定周期でループする。
  double current_valueL, current_valueR; //現在の出力値
  static double last_valueL=0.0; //一つ前の出力値（要保存のためstatic）
  static double last_valueR=0.0;
  double errorL, errorR, d_errorL, d_errorR; //偏差、偏差の微小変化量
  static double i_errorL=0.0; //偏差の総和（要保存のためstatic
  static double i_errorR=0.0;
  float distance = 0.0;
  double dt; 
  
  dt = getDt();
  current_valueL = 1/time00;
  current_valueR = 1/time10;
  
  distance += PI*WHEEL_R*(current_valueL + current_valueR);
  
  errorL = target_value - current_valueL; //偏差の計算
  errorR = target_value - current_valueR;
  
  d_errorL = current_valueL - last_valueL; //偏差微小変化量の計算
  d_errorR = current_valueR - last_valueR;
  
  control_valueL = p_gain*errorL + i_gain*i_errorL + d_gain*d_errorL;
  control_valueR = p_gain*errorR + i_gain*i_errorR + d_gain*d_errorR;
  
  motor_controlVolt(control_valueL, control_valueR);
   //制御量の計算 
  last_valueL = current_valueL; //一つ前の出力値を更新
  last_valueR = current_valueR;
  
  i_errorL += errorL; //偏差の総和を更新
  i_errorR += errorR;
  
  return(distance);
} 

float Run::batt_voltage(void)
{
  float batt_voltage = 0.0;
  
  batt_voltage = (analogRead(A1) - 4.0) / 100.0;
  return(batt_voltage);
}

double Run::getDt(void)
{
  static unsigned long int lastTime=0.0;
  
  long nowTime = micros();
  double time = (double)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;
  
  return( time );
}

ECEF Run::GEDE2ECEF(GEDE cod_f, GEDE cod)
{
  ECEF tmp;
  
  cod.LAT = cod.LAT/(float)180*PI;
  cod_f.LAT = cod_f.LAT/(float)180*PI;
  cod.LON = cod.LON/(float)180*PI;
  cod_f.LON = cod_f.LON/(float)180*PI;
  tmp.X = ( N(cod.LAT) + cod.AGE ) * cos(cod.LAT) * cos(cod.LON);
  tmp.Y = ( N(cod.LAT) + cod.AGE ) * cos(cod.LAT) * sin(cod.LON);
  tmp.Z = (long unsigned int)( (N(cod.LAT) * (1 - E_2) + cod.AGE ) * sin(cod.LAT) );
  tmp.X_F = ( N(cod_f.LAT) + cod.AGE ) * cos(cod_f.LAT) * cos(cod_f.LON);
  tmp.Y_F = ( N(cod_f.LAT) + cod.AGE ) * cos(cod_f.LAT) * sin(cod_f.LON);
  tmp.Z_F = tmp.Z;     
  return(tmp);
}

ENU Run::ECEF2ENU(ECEF vector, GEDE cod)
{
  ENU tmp;
  long unsigned int num1,num2,num3;
  int a,b,c;
  
  cod.LAT = cod.LAT/(float)180*PI;
  cod.LON = cod.LON/(float)180*PI;
  tmp.E = -sin(cod.LON)*(vector.X - vector.X_F) + cos(cod.LON)*(vector.Y - vector.Y_F);
  tmp.N = -sin(cod.LAT)*cos(cod.LON)*(vector.X - vector.X_F) - sin(cod.LAT)*sin(cod.LON)*(vector.Y - vector.Y_F) + cos(cod.LAT)*(vector.Z - vector.Z_F);
  a = cos(cod.LAT)*cos(cod.LON)*(vector.X - vector.X_F);
  b = num1 = cos(cod.LAT)*sin(cod.LON)*(vector.Y - vector.Y_F);
  c = sin(cod.LAT)*(vector.Z - vector.Z_F);
  if(a<=0)
    a = 0;
  
  if(b<=0)
    b = 0;
  
  if(c<=0)
    c = 0;

  num1 = (long unsigned int)a;
  num2 = (long unsigned int)b;
  num3 = (long unsigned int)c;

  tmp.U = num1 + num2 + num3;
  return(tmp);
}

ENU Run::GEDE2ENU(GEDE cod_f, GEDE cod)
{
  ECEF tmp_ecef;
  ENU tmp_enu;
  
  tmp_ecef = GEDE2ECEF(cod_f, cod);
  tmp_enu = ECEF2ENU(tmp_ecef, cod);
   
  return(tmp_enu);
}

double Run::kalmanFilter_DistanceX(double accel, double distance, double dt)
{
  static double x[2] = {0.0, 0.0};
  static double P[2][2] = { {0, 0}, {0, 0}};
  static double K[2];
  const  double Q[2][2] = { {0.01, 0}, {0, 0.003}}; 
  const  double R = 1.0;
    
  x[1] = x[1] + 9.8*accel*dt;
  x[0] = x[0] + x[1]*dt;
  
  P[0][0] += dt*(P[1][0] + dt*(P[0][1] + dt*P[1][1]) + Q[0][0]);
  P[0][1] += dt*P[1][1];
  P[1][0] += dt*P[1][1];
  P[1][1] += dt*Q[1][1];
  
  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);
  
  x[0] += K[0]*(distance - x[0]);
  x[1] += K[1]*(distance - x[0]);
    
  P[0][0] -= P[0][0]*K[0];
  P[0][1] -= P[0][1]*K[0];
  P[1][0] -= K[1]*P[0][0];
  P[1][1] -= K[1]*P[0][1];
  
  return(x[0]);
}

double Run::kalmanFilter_DistanceY(double accel, double distance, double dt)
{
  static double x[2] = {0.0, 0.0};
  static double P[2][2] = { {0, 0}, {0, 0}};
  static double K[2];
  const  double Q[2][2] = { {0.01, 0}, {0, 0.003}}; 
  const  double R = 1.0;
    
  x[1] = x[1] + 9.8*accel*dt;
  x[0] = x[0] + x[1]*dt;
  
  P[0][0] += dt*(P[1][0] + dt*(P[0][1] + dt*P[1][1]) + Q[0][0]);
  P[0][1] += dt*P[1][1];
  P[1][0] += dt*P[1][1];
  P[1][1] += dt*Q[1][1];
  
  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);
  
  x[0] += K[0]*(distance - x[0]);
  x[1] += K[1]*(distance - x[0]);
    
  P[0][0] -= P[0][0]*K[0];
  P[0][1] -= P[0][1]*K[0];
  P[1][0] -= K[1]*P[0][0];
  P[1][1] -= K[1]*P[0][1];
  
  return(x[0]);
}


