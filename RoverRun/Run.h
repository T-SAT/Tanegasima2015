#ifndef _RUN_H_INCLUDED_
#define _RUN_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ROVER_R     0.09 //[m]
#define WHEEL_R     0.07 //[m]

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
  ENU enu1;
  ENU enu2;
} ENUs;

class Run{
  public :
    void ADCInit(void);
    void motorInit(int LF, int LB, int RF, int RB);
    void rollInit(void);
    
  public :
    float get_distance(void);
    float get_angle(void);
    float get_gyro(void);
    void get_line(void);
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
  /*
  public :
    ECEF GEDE2ECEF(GEDE cod_f, GEDE cod);
    ENU ECEF2ENU(ECEF vector, GEDE cod);
    ENU GEDE2ENU(GEDE cod_f, GEDE cod);
    
  public :
    int turnDecision(void);
    float crossProduct(void);
    float checkAngle(ENU enu1, ENU enu2);
    */
  private :
    int motorPinLF;
    int motorPinLB;
    int motorPinRF;
    int motorPinRB;
    float line_angle;
   
  private :
    float origin[2];
    float curent[2];
    
};

extern Run run;

#endif
