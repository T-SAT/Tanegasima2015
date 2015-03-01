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

  Serial.print(ENABLE);
  Serial.setintr(Slave.receive_data);
}

void loop() { 
  //⊿tの測定 bh

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
  delay(2000);
  digitalWrite(PARA_PIN, LOW);
}



