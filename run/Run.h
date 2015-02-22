#ifndef _RUN_H_INCLUDED_
#define _RUN_H_INCLUDED_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Run{
  public :
    void ADCInit(void);
    void motorInit(int EN, int LF, int LB, int RF, int RB);
    
  public :
    float get_distance(void);
    float get_angle(void);
    float get_gyro(void);
    void motor_control(int motorL, int motorR);
    void motor_controlVolt(float motorL, float motorR);
    void turn(float angle);
    void steer(void);
    
  public :
    float batt_voltage(void);
    double getDt(void);
    
  private :
    int motorPinLF;
    int motorPinLB;
    int motorPinRF;
    int motorPinRB;
    int motorENPin;
  
};

extern Run run;

#endif
