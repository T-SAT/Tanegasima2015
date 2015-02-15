//com10: master
#include <Wire.h>
#include "SerialMaster.h"

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  Master.request_data(GYRO_NUM);
}

