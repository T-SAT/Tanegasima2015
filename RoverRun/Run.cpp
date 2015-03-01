#include "Run.h"
#include "SerialMaster.h"
#include <SD.h>
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

float Run::get_lineDistance(void)
{
  float tmp;

  tmp = rover.distance*cos(PI - rover.angle) + (tan(rover.angle - goal.angle))/(rover.distance*sin(rover.angle - PI/2));
  return(tmp);
}

float Run::get_lineAngle(void)
{
  return(rover.angle - goal.angle);
}


float Run::get_lineGyro(void)
{
  float gyro;

  Master.request_data(GYRO_NUM);
  gyro = Master.get(GYRO_NUM,'z');
  return(last_targetValue - gyro);
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

float Run::get_targetValue(void)
{
  return(last_targetValue);
}

int Run::crossProduct(float vec1X, float vec1Y, float vec2X, float vec2Y)
{
  float tmp;

  tmp = vec1X*vec2Y - vec2X*vec1Y;
  if(tmp < 0) return(1);
  else if(0 < tmp) return(-1);
  else return(0);
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

  time = micros();
  ;
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
  if(motorL>0 && motorR<0){
    digitalWrite(motorPinRF, LOW);
    analogWrite(motorPinRB, -motorR);
    analogWrite(motorPinLF, motorL);
    digitalWrite(motorPinLB, LOW);
  }

  else if(motorL<0 && motorR>0){   
    analogWrite(motorPinRF, motorR);
    digitalWrite(motorPinRB, LOW);
    digitalWrite(motorPinLF, LOW);
    analogWrite(motorPinLB, -motorL);
  }

  else if(motorR<0 && motorL<0){
    digitalWrite(motorPinRF, LOW);
    analogWrite(motorPinRB, -motorR);
    digitalWrite(motorPinLF, LOW);
    analogWrite(motorPinLB, -motorL);
  }
  else {
    analogWrite(motorPinRF, motorR);
    digitalWrite(motorPinRB, LOW);
    analogWrite(motorPinLF, motorL);
    digitalWrite(motorPinLB, LOW);
  } 
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
  } 
  while(0 < error);

}

void Run::steer(float current_value, float target_value)
{
  const double p_gain=1.0; //Pゲイン
  const double i_gain=0.001;//0.001; //Iゲイン（多すぎると暴走します）
  const double d_gain=0.5; //Dゲイン
  double control_value=0.0;
  double control_valueL=0.0; //制御量（モータなどへの入力量）
  double control_valueR=0.0;
  //割り込みタイマー処理(H8ならITUなど。一定周期でwループする。
  static double last_value = 0.0; //一つ前の出力値（要保存のためstatic）
  double error, d_error; //偏差、偏差の微小変化量
  static double i_error=0.0; //偏差の総和（要保存のためstatic

  error = target_value - current_value; //偏差の計算
  d_error = current_value - last_value;
  control_value = p_gain*error + i_gain*i_error + d_gain*d_error;
  control_valueL = (control_value < 0 ? -1 : 1)*(constrain(abs(control_value), 3, 127));
  control_valueR = (control_value < 0 ? -1 : 1)*(constrain(abs(control_value), 3, 127) - 2);  
  Master.saveLog("c_value:", current_value, millis());
  Master.saveLog("dutyL :", 125.0 - control_value, millis());
  Master.saveLog("dutyR :", 125.0 + control_value, millis()); 
  motor_control(127-control_valueL, 127+control_valueR);
  //制御量の計算 
  last_value = current_value; //一つ前の出力値を更新
  i_error += error; //偏差の総和を更新
}

float Run::batt_voltage(void)
{
  float batt_voltage = 0.0;

  batt_voltage = (analogRead(A0) - 4.0) / 100.0;
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

void Run::update_PolarCoordinates(float d, float Dseata)
{
  //float Dseata = angle - rover.angle;

  rover.distance = rover.distance*cos(Dseata) + sqrt(pow(rover.distance, 2)*(pow(cos(Dseata), 2) - 1) + pow(d, 2));
  rover.angle += Dseata;
}

void Run::update_targetValue(float gyro, double dt)
{
  const double Kd=1.0; //Pゲイン
  const double Ka=0.001; //Iゲイン（多すぎると暴走します）
  const double Kg=0.5; //Dゲイン

  last_targetValue = gyro + dt*(-Kd*get_lineDistance() - Ka*get_lineAngle() - Kg*get_lineGyro());
}

ECEF Run::GEDE2ECEF(GEDE cod, double height)
{
  ECEF ecef;

  ecef.X = (NN(cod.LAT)+height)*cos(cod.LAT*PI/180)*cos(cod.LON*PI/180);
  ecef.Y = (NN(cod.LAT)+height)*cos(cod.LAT*PI/180)*sin(cod.LON*PI/180);
  ecef.Z = (NN(cod.LAT)*(1-E2)+height)*sin(cod.LAT*PI/180);
  return ecef;
}

GEDE Run::ECEF2GEDE(ECEF ec)
{
  GEDE blh;
  int i = 0;
  double phi, ramda, height, p;
  double x, y, z;
  double sita;

  x = ec.X, y = ec.Y, z = ec.Z;
  p = sqrt(x*x + y*y);
  sita = (180/PI) * atan2(z*A, p*B);
  /*--- 緯度 */
  phi = (180/PI) * atan2(z+ED2*B*(pow(sin(sita*PI/180), 3)),(p-E2*A*(pow(cos(sita*PI/180), 3))));
  /*--- 経度 */
  ramda = (180/PI) * atan2(y,x);
  /*--- 高さ */
  height = (p / cos(phi*PI/180)) - NN(phi);
  blh.LAT = phi; 
  blh.LON = ramda; 
  return(blh);
}

ENU Run::ECEF2ENU(ECEF origin, ECEF dest)
{
  int i, j;
  GEDE blh;
  ECEF mov;
  ENU ret;
  double rotyp[3][3], rotzp1[3][3], rotzp2[3][3];
  double mat_conv1[3][3] = {
  };
  double mat_conv2[3][3] = {
  };

  blh = ECEF2GEDE(origin);
  rotz(rotzp1,90.0);
  roty(rotyp, 90.0 - blh.LAT);
  rotz(rotzp2,blh.LON);
  matmat(mat_conv1, rotzp1, rotyp);
  matmat(mat_conv2, mat_conv1, rotzp2);
  mov.X = dest.X - origin.X;
  mov.Y = dest.Y - origin.Y;
  mov.Z = dest.Z - origin.Z;
  ret = matvec(mat_conv2, mov);

  return ret;
}

ENU Run::GEDE2ENU(GEDE origin, GEDE dest, double high)
{
  ECEF ecef, ecef_o;
  ENU tmp_enu;

  ecef_o = GEDE2ECEF(origin, high);
  ecef = GEDE2ECEF(dest, high);
  tmp_enu = ECEF2ENU(ecef_o, ecef);

  return(tmp_enu);
}

void Run::setCoordinates(char *str, float flat, float flon)
{
  if(str == "origin"){
    ORIGIN.LAT = flat;
    ORIGIN.LON = flon;
  } 
  else if(str == "goal"){
    GOAL.LAT = flat;
    GOAL.LON = flon;
  } 
  else if(str == "landing"){
    LANDING.LAT = flat;
    LANDING.LON = flon;
  } 
}

void Run::setENU(char *str, ENU enu)
{
  if(str == "origin"){
    originENU.E = enu.E;
    originENU.N = enu.N;
    originENU.U = enu.U;
  } 
  else if(str == "goal"){
    goalENU.E = enu.E;
    goalENU.N = enu.N;
    goalENU.U = enu.U;
  }

}

void Run::setPolarCoordinates(char *str, float distance, float angle)
{
  if(str == "goal"){
    goal.distance = distance;
    goal.angle = angle;
  } 
  else if(str == "rover"){
    rover.distance = distance;
    goal.angle = angle;
  }
}

PolarCoordinates Run::ENU2PolarCoordinates(ENU enu1, ENU enu2)
{
  float angle, distance;
  PolarCoordinates tmp;

  angle = get_angle(enu1.E, enu1.N, enu2.E, enu2.N);
  distance = sqrt(pow(enu2.E, 2) + pow(enu2.N, 2));
  tmp.distance = distance;
  tmp.angle = angle;

  return(tmp);
}

ENU Run::PolarCoordinates2ENU(PolarCoordinates polar)
{
  ENU tmp;

  tmp.E = polar.distance * cos(polar.angle*DEG_TO_RAD);
  tmp.N = polar.distance * sin(polar.angle*DEG_TO_RAD);

  return(tmp);
}

float Run::distanceOFgoal2current(GEDE current)
{
  ENU tmp;

  tmp = GEDE2ENU(current, GOAL, HEIGHT);

  return(sqrt(pow(tmp.E, 2) + pow(tmp.N, 2)));
}

void Run::improveCurrentCoordinates(GEDE current)
{
  ENU current_correct, current_polar;
  float distance;
  ENU tmp;

  current_correct = GEDE2ENU(ORIGIN, current, HEIGHT);
  current_polar = PolarCoordinates2ENU(rover);
  distance = sqrt(pow(current_correct.E - current_polar.E, 2) + pow(current_correct.N - current_polar.N, 2));

  if(max(distance, -distance) > 10.0){
    tmp = GEDE2ENU(ORIGIN, current, HEIGHT);
    rover = ENU2PolarCoordinates(originENU, tmp);
  }
}

double Run::kalmanFilter_DistanceX(double accel, double distance, double dt)
{
  static double x[2] = {
    0.0, 0.0                                  };
  static double P[2][2] = { 
    {
      0, 0                                                                    }
    , {
      0, 0                                                                    }
  };
  static double K[2];
  const  double Q[2][2] = { 
    {
      0.01, 0                                                                    }
    , {
      0, 0.003                                                                    }
  }; 
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
  static double x[2] = {
    0.0, 0.0                                  };
  static double P[2][2] = { 
    {
      0, 0                                                                    }
    , {
      0, 0                                                                    }
  };
  static double K[2];
  const  double Q[2][2] = { 
    {
      0.01, 0                                                                    }
    , {
      0, 0.003                                                                    }
  }; 
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

void Run::rotx(double rota[3][3], double sita)
/* x軸回りのsita度の回転変換：右ねじの方向 */
{
  rota[0][0] = 1;
  rota[0][1] = 0;
  rota[0][2] = 0;
  rota[1][0] = 0;
  rota[1][1] = cos(sita*PI/180.0);
  rota[1][2] = sin(sita*PI/180.0);
  rota[2][0] = 0;
  rota[2][1] = -sin(sita*PI/180.0);
  rota[2][2] = cos(sita*PI/180.0);
}

void Run::roty(double rota[3][3], double sita)
{
  rota[0][0] = cos(sita*PI/180.0);
  rota[0][1] = 0;
  rota[0][2] = -sin(sita*PI/180.0);
  rota[1][0] = 0;
  rota[1][1] = 1;
  rota[1][2] = 0;
  rota[2][0] = sin(sita*PI/180.0);
  rota[2][1] = 0;
  rota[2][2] = cos(sita*PI/180.0);
}

void Run::rotz(double rota[3][3], double sita)
{
  rota[0][0] = cos(sita*PI/180.0);
  rota[0][1] = sin(sita*PI/180.0);
  rota[0][2] = 0;
  rota[1][0] = -sin(sita*PI/180.0);
  rota[1][1] = cos(sita*PI/180.0);
  rota[1][2] = 0;
  rota[2][0] = 0;
  rota[2][1] = 0;
  rota[2][2] = 1;
}


void Run::matmat(double c[NUM][NUM],double a[NUM][NUM],double b[NUM][NUM])
{
  int i,j,k;
  double r,s,t;

  r = 0;
  //受け取った２つの行列の掛け算を行う。
  for(i=0;i<NUM;i++) {
    for(j=0;j<NUM;j++) {
      for(k=0;k<NUM;k++) {
        t = c[i][j] + (a[i][k]*b[k][j] + r);
        r = (a[i][k]*b[k][j] + r) - (t - c[i][j]);
        c[i][j] = t;
      }
      r = 0;
    }
    r = 0;
  }

}



ENU Run::matvec(double mat[3][3], ECEF vector)
{
  ENU tmp;

  tmp.E = mat[0][0]*vector.X + mat[0][1]*vector.Y + mat[0][2]*vector.Z;
  tmp.N = mat[1][0]*vector.X + mat[1][1]*vector.Y + mat[1][2]*vector.Z;
  tmp.U = mat[2][0]*vector.X + mat[2][1]*vector.Y + mat[2][2]*vector.Z;

  return(tmp);
}

















