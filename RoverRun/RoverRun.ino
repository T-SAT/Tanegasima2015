#include <Wire.h>
#include <SD.h>
#include <wiring_private.h>
#include "Run.h"
#include "SerialMaster.h"
#include <SoftwareSerial.h>

void setup()
{
  Serial.begin(9600);
  run.motorInit(2, 3, 4, 5);
  //run.rollInit();
  Wire.begin();
}

void loop()
{
  float accXval;
  
  Master.request_data(ALL_NUM);
  accXval = Master.get(ACCEL_NUM, 'x');
  Serial.println(accXval);
}
