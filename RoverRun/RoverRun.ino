#include <Wire.h>
#include <SD.h>
#include <wiring_private.h>
#include "Run.h"
#include "SerialMaster.h"
#include <SoftwareSerial.h>

File saveFile;

void setup()
{
  Serial.begin(9600);
  run.motorInit(2, 3, 4, 5);
  //run.rollInit();
  pinMode(10, OUTPUT);
  if(!SD.begin(8)) {
    Serial.println("initialization failed!");
  }

  Wire.begin();
}

void loop()
{
  GEDE tmp1, tmp2;
  ENU enu;
  ECEF ecef, ecef_o;
  double high1, high2;
 
  tmp1.LAT = 38.14227288;
  tmp1.LON = 140.93265738;
  high1 = 45.664;
  
  tmp2.LAT = 38.13877338;
  tmp2.LON = 140.89872429;
  high2 = 44.512;
  
  ecef = run.GEDE2ECEF(tmp1, high1);
  ecef_o = run.GEDE2ECEF(tmp2, high2);
  enu = run.ECEF2ENU(ecef_o, ecef);
  
  Serial.print(enu.E);  Serial.print('\t'); 
  Serial.print(enu.N);  Serial.print('\t'); 
  Serial.println(enu.U);  
  delay(1000);
  
}


