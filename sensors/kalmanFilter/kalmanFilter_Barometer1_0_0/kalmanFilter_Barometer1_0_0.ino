#include <Wire.h>
#include <LPS331.h>

LPS331 ps;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
    if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }

  ps.enableDefault();
}

void loop()
{
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
  
  Serial.print("p: ");
  Serial.print(pressure);
  Serial.print(" mbar\ta: ");
  Serial.print(altitude);
  Serial.print(" m\tt: ");
  Serial.print(temperature);
  Serial.println(" deg C");

  delay(100);
}

double kalmanFilter_Barometer(double accel, double advanced, double init_advanced, double dt)
{
  static double x[2] = {init_advanced, 0};
  static double P[2][2] = { {0, 0}, {0, 0} };
  static double K[2];
  const double Q[2][2] = { {0.01, 0}, {0, 0.003} };
  const double R = 1;
  
  x[0] += dt * (u - x[1]);

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q[1][1] * dt;
    
  K[0] = P[0][0] / (P[0][0] + R);
  K[1] = P[1][0] / (P[0][0] + R);

  x[0] += K[0] * (y - x[0]);
  x[1] += K[1] * (y - x[0]);

  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
    
  return x[0];
}

double getDt(void)
{
  static long lastTime=0;
  
  long nowTime = micros();
  double time = (double)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;
  
  return( time );  
}

