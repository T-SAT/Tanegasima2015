#ifndef _RUN_H_INCLUDED_
#define _RUN_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define HEIGHT   10.0

#define ROVER_R     0.09 //[m]
#define WHEEL_R     0.07 //[m]

#define PI 3.1415926535898
#define A 6378137.0 /* Semi-major axis */
#define ONE_F 298.257223563 /* 1/F */
#define B (A*(1.0 - 1.0/ONE_F))
#define E2 ((1.0/ONE_F)*(2-(1.0/ONE_F)))
#define ED2 (E2*A*A/(B*B))
#define NN(p) (A/sqrt(1.0 - (E2)*pow(sin(p*PI/180.0), 2)))
#define NUM  3

typedef struct {
  double matrix3TO3[3][3];
  double vector3TO1[3];         //Z軸との型が違うのでキャストしとけ
} 
VECTOR;

typedef struct {
  float LAT;
  float LON;
} 
GEDE;

typedef struct {
  double X;
  double Y;
  double Z;
} 
ECEF;

typedef struct {
  double E;
  double N;
  long unsigned int U;
} 
ENU;

typedef struct {
  float distance;
  float angle;  //[deg]
} 
PolarCoordinates;

class Run{
public :
  void ADCInit(void);
  void motorInit(int LF, int LB, int RF, int RB);
  void rollInit(void);

public :
  float get_lineDistance(void);
  float get_lineAngle(void);
  float get_lineGyro(void);
  float get_angle(float vec1X, float vec1Y, float vec2X, float vec2Y);
  float get_targetValue();
  int crossProduct(float vec1X, float vec1Y, float vec2X, float vec2Y);
  unsigned long int rollInstrumentL(void);
  unsigned long int rollInstrumentR(void);
  static void get_rollInstrumentL(void);
  static void get_rollInstrumentR(void);

public :
  void motor_control(int motorL, int motorR);
  void motor_controlVolt(float motorL, float motorR);
  void turn(float target_value);
  void steer(float current_value, float target_value);

public :
  float batt_voltage(void);
  double getDt(void);

public :
  void update_PolarCoordinates(float d, float Dseata);
  void update_targetValue(float gyro, double dt);

public :
  ECEF GEDE2ECEF(GEDE cod, double height);
  GEDE ECEF2GEDE(ECEF ec);
  ENU ECEF2ENU(ECEF origin, ECEF dest);
  ENU GEDE2ENU(GEDE origin, GEDE dest, double high);
  void setCoordinates(char *str, float flat, float flon);
  void setENU(char *str, ENU enu);
  void setPolarCoordinates(char *str, float distance, float angle);
  PolarCoordinates ENU2PolarCoordinates(ENU enu1, ENU enu2);
  ENU  PolarCoordinates2ENU(PolarCoordinates tmp);

public :
  float distanceOFgoal2current(GEDE current);
  void improveCurrentCoordinates(GEDE current);

public :
  double kalmanFilter_DistanceX(double accel, double distance, double dt);
  double kalmanFilter_DistanceY(double accel, double distance, double dt);

public:
  void rotx(double rota[3][3], double sita);  //deg
  void roty(double rota[3][3], double sita);  //deg
  void rotz(double rota[3][3], double sita);  //deg
  void matmat(double tmp[3][3], double mat1[3][3], double mat2[3][3]);
  ENU matvec(double mat[3][3], ECEF vector);

public:
  float last_targetValue;

public :
  GEDE ORIGIN;
  GEDE GOAL;
  GEDE LANDING;
  PolarCoordinates rover;
  PolarCoordinates goal;
  ENU originENU;
  ENU goalENU;
  
private :
  int motorPinLF;
  int motorPinLB;
  int motorPinRF;
  int motorPinRB;
};

extern Run run;

#endif

