//com12: slave
#include <Wire.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <LPS331.h>
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"

LPS331 ps;
/*
SoftwareSerial ss(12, 11);
TinyGPS gpsSerial;
*/

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
  sensorData tmp;
  unsigned long int time;
  int check_sd;

  delay(200);
}

void sensorInit()
{
  IMU.sensorInit();
  ps.init();
  ps.enableDefault();
}

void cut_parachute(void)
{
  pinMode(PARA_PIN, OUTPUT);
  digitalWrite(PARA_PIN, HIGH);
  delay(2000);
  digitalWrite(PARA_PIN, LOW);
}

/*
float recvGPS(char select)
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed; 

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  { 
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gpsSerial.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gpsSerial.f_get_position(&flat, &flon, &age);
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    return(select == 'x' ? flat : flon);
  }
}  

*/
