#include <Wire.h>

#define DEVICE (0x53) // Device address as specified in data sheet 
#define TO_READ 6 // 2 bytes for each axis x, y, z 
 
byte _buff[6];
 
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

#define GYRO_ADDR 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

float AccZero[3] = {0};
float GyrZero[4] = {0};
float MagZero[3] = {0};

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output. Make sure you set your Serial Monitor to the same!
 
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(POWER_CTL, 0x08);
  sensorInit();
}

void loop()
{
  float dt = getDt();
  int xA, yA, zA, xG, yG, zG;
  float accX, accY, accZ, gyroX, gyroY, gyroZ;
  static float angle_gyro = 180;
  float angle_acc = 180;
  float angle_kalman;
  
  readAccel(&xA, &yA, &zA); 
  getGyroscopeData(&xG, &yG, &zG);
  
  accX = (float)xA - AccZero[0];
  accY = (float)yA - AccZero[1];
  accZ = (float)zA - AccZero[2];
  gyroX = (float)xG - GyrZero[0];
  gyroY = (float)yG - GyrZero[1]; 
  gyroZ = (float)zG - GyrZero[2];
  
  angle_gyro += gyroX * dt;
  angle_acc += atan2(accY, accZ);
  angle_kalman = kalmanFiltering(gyroX, angle_acc, dt);
  
  Serial.print(angle_gyro);  Serial.print('\t');
  Serial.print(angle_acc);   Serial.print('\t');
  Serial.print(angle_kalman);Serial.print('\t');
  Serial.print('\n');
}


float kalmanFiltering(float u, float y, float dt)
{
  static float x[2] = {180, 0};
  static float P[2][2] = { {0, 0}, {0, 0} };
  static float K[2];
  const float Q[2][2] = { {0.01, 0}, {0, 0.003} };
  const float R = 1;
  
  x[0] += dt * (u - x[1]);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
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
  
  return(x[0]);
}

float getDt(void)
{
  static long lastTime=0;
  long nowTime = micros();
  float time = (float)(nowTime - lastTime);
  
  time = max(time, 20);
  time /= 1000000;
  lastTime = nowTime;
  
  return(time);
}

void sensorInit(void)
{
  float accZero[3] = {0};
  float gyrZero[4] = {0};
  float magZero[3] = {0};
  int xA, yA, zA, xG, yG, zG;
  int i;
  
  initGyro ();
  delay(1000);
  for(i=0; i<100; i++){
    readAccel(&xA, &yA, &zA); // read the x/y/z tilt
    getGyroscopeData(&xG, &yG, &zG); 
    accZero[0] += (float)xA;
    accZero[1] += (float)yA;
    accZero[2] += (float)zA;
    gyrZero[0] += (float)xG;
    gyrZero[1] += (float)yG;
    gyrZero[2] += (float)zG;
    delay(10);
  }
  
  accZero[0] /= 100.0;  
  accZero[1] /= 100.0;
  accZero[2] /= 100.0;
  gyrZero[0] /= 100.0;
  gyrZero[1] /= 100.0;
  gyrZero[2] /= 100.0;

  accZero[2] -= 1; 
  
  AccZero[0] = accZero[0];
  AccZero[1] = accZero[1];
  AccZero[2] = accZero[2];
  GyrZero[0] = GyrZero[0];
  GyrZero[1] = GyrZero[1];
  GyrZero[2] = GyrZero[2];
} 
  
