#ifndef _RUN_H_INCLUDED_
#define _RUN_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ROVER_R     0.09 //[m]
#define WHEEL_R     0.07 //[m]

#define REDLINE_R 6378137
#define E_2 0.00335281*2 - 0.00335281*0.00335281
#define N(a) 6378137 / sqrt( 1.0 - (0.00335281*2 - 0.00335281*0.00335281)*sin(a)*sin(a) )

typedef struct {
  double matrix3TO3[3][3];
  double vector3TO1[3];         //Z軸との型が違うのでキャストしとけ
} VECTOR;

typedef struct {
  float LAT;
  float LON;
  long unsigned int AGE;
} GEDE;

typedef struct {
  double X;
  double Y;
  unsigned long Z;
  double X_F;
  double Y_F;
  long unsigned int Z_F;
} ECEF;

typedef struct {
  double E;
  double N;
  long unsigned int U;
} ENU;

typedef struct {
  float angle;
  float distance;
} PolarCoordinates;

class Run{
  public :
    void ADCInit(void);
    void motorInit(int LF, int LB, int RF, int RB);
    void rollInit(void);
    
  public :
    void get_distance(void);
    void get_angle(void);
    float get_angle(float vec1X, float vec1Y, float vec2X, float vec2Y);
    int crossProduct(float vec1X, float vec1Y, float vec2X, float vec2Y);
    float get_gyro(void);
    unsigned long int rollInstrumentL(void);
    unsigned long int rollInstrumentR(void);
    static void get_rollInstrumentL(void);
    static void get_rollInstrumentR(void);
    
  public :
    void motor_control(int motorL, int motorR);
    void motor_controlVolt(float motorL, float motorR);
    void turn(float target_value);
    void steer(void);
    float forward(float target_value); //target_value -> rps
    
  public :
    float batt_voltage(void);
    double getDt(void);
  
  public :
    ECEF GEDE2ECEF(GEDE cod_f, GEDE cod);
    ENU ECEF2ENU(ECEF vector, GEDE cod);
    ENU GEDE2ENU(GEDE cod_f, GEDE cod);
    PolarCoordinates ENU2PolarCoordinates(ENU enu1, ENU enu2);
    
  public :
    double kalmanFilter_DistanceX(double accel, double distance, double dt);
    double kalmanFilter_DistanceY(double accel, double distance, double dt);
    
  public :
    int turnDecision(void);
    float crossProduct(void);
    float checkAngle(ENU enu1, ENU enu2);
   
  private :
    int motorPinLF;
    int motorPinLB;
    int motorPinRF;
    int motorPinRB;
    float line_angle;
   
  private :
    GEDE ORIGIN;
    GEDE GOAL;
    PolarCoordinates rover;
    PolarCoordinates goal;
};

extern Run run;

#endif
