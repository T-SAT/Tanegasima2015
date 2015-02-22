#include "Run.h"
#include "SerialMaster.h"

Run run;

void Run::ADCInit(void)
{
  DDRC &= ~_BV(5);
}

void Run::motorInit(int EN, int LF, int LB, int RF, int RB)
{
  motorPinLF = LF;
  motorPinLB = LB;
  motorPinRF = RF;
  motorPinRB = RB;
  motorENPin = EN;
  
  pinMode(motorENPin, OUTPUT);
  pinMode(motorPinLF, OUTPUT);
  pinMode(motorPinLB, OUTPUT);
  pinMode(motorPinRF, OUTPUT);
  pinMode(motorPinRB, OUTPUT);
}
  
float Run::get_distance(void)
{
}

float Run::get_angle(void)
{
}

float Run::get_gyro(void)
{
}

void Run::motor_control(int motorL, int motorR)
{
}

void Run::motor_controlVolt(float motorL, float motorR)
{
}

void Run::turn(float target_value)
{
  double p_gain=1.0; //Pゲイン
  double i_gain=0.001; //Iゲイン（多すぎると暴走します）
  double d_gain=0.5; //Dゲイン
  double control_value=0.0; //制御量（モータなどへの入力量）

//割り込みタイマー処理(H8ならITUなど。一定周期でループする。)
  double current_value; //現在の出力値
  double last_value=0.0; //一つ前の出力値（要保存のためstatic）
  double error,d_error; //偏差、偏差の微小変化量
  double i_error=0.0; //偏差の総和（要保存のためstatic）
  double dt;

  do{
    dt = run.getDt();
    Master.request_data(GYRO_NUM);
    current_value = Master.get(GYRO_NUM, 'z')*dt;

    error = target_value-current_value; //偏差の計算
    d_error = current_value-last_value; //偏差微小変化量の計算

    control_value= p_gain*error + i_gain*i_error + d_gain*d_error;
   //制御量の計算
    if(target_value < 0)
      motor_controlVolt(-control_value, control_value);
    else 
      motor_controlVolt(control_value, -control_value);

    last_value=current_value; //一つ前の出力値を更新
    i_error+=error; //偏差の総和を更新
  }while(0 < error );
}

void Run::steer(void)
{
}

float Run::batt_voltage(void)
{
  float batt_voltage = 0.0;
  
  batt_voltage = (analogRead(A1) - 4.0) / 100.0;
  return(batt_voltage);
}

double Run::getDt(void)
{
  static long lastTime=0;
  
  long nowTime = micros();
  double time = (double)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;
  
  return( time );
}

