//com12: slave
#include <Wire.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <LPS331.h>
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"

LPS331 ps;

void setup() {
  float pressure, tempreture;
  float flat,flon;

  Serial.begin(9600); 
  sensorInit();
  pinMode(10,OUTPUT);

  pressure = ps.readPressureMillibars();
  tempreture = ps.readTemperatureC();
  ps.set_initValue(pressure, tempreture);
  Slave.saveRadio_begin(9600);
  fall();
  
  Serial.print(ENABLE);
  Serial.setintr(Slave.receive_data);
}

void loop() { 
  delay(50);
}

void sensorInit()
{
  IMU.sensorInit();
  ps.init();
  ps.enableDefault();
  Slave.saveRadio_begin(9600);
}

void cut_parachute(void)
{
  pinMode(PARA_PIN, OUTPUT);
  digitalWrite(PARA_PIN, HIGH);
  delay(1000);
  digitalWrite(PARA_PIN, LOW);
}

void fall(void)
{
  float altitude;
  float pressure;
  int fallFlag1 = 1;
  int  fallFlag2 = 1;
  unsigned long int time;
  
  time = millis();
  do {
    pressure = ps.readPressureMillibars();
    altitude = ps.pressureToAltitudeMeters(pressure);
    Slave.saveRadio(altitude, millis());
    if(30.0 <= altitude)
      fallFlag1 = 0;
    
    if(altitude <= 20.0 && !fallFlag1)
      Serial.print(MOTOR_START);
      
    if(altitude <= 1.0 && !fallFlag1)
      fallFlag2 = 0;
  } while(fallFlag1 || fallFlag2 && millis()-time <= 240000);
  cut_parachute();
  Serial.print(MOTOR_END);
}





