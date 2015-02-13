//com10: master
#include "SerialMaster.h"
#include <Wire.h>

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  delay(5000);
}

void loop()
{
  Master.request_data(ACCEL_NUM);
}
