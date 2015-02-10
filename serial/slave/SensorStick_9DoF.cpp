#include <Wire.h>
#include <TinyGPS.h>
#include <SD.h>

#include "SensorStick_9DoF.h"

SensorStick_9DoF IMU;
  
void SensorStick_9DoF::begin(){
	Wire.begin();
	
	//ADXL345 PowerOn
	twiWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, 0);      
	twiWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, 16);
	twiWrite(ADXL345_ADDRESS, ADXL345_POWER_CTL, 8); 
        twiWrite(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 0x03);   //g Range set (+-)16g
	
	//ITG3200 Setup
	twiWrite(ITG3200_ADDRESS, ITG3200_PWR_MGM, 0x00);
	twiWrite(ITG3200_ADDRESS, ITG3200_SMPLRT_DIV , 0x04);  //5msec
	twiWrite(ITG3200_ADDRESS, ITG3200_DLPF_FS , 0x1E);
        twiWrite(ITG3200_ADDRESS, ITG3200_INT_CFG, 0x00);
	
	//HMC5883L Setup
	twiWrite(HMC5883L_ADDRESS, HMC5883L_CONFIGURATION_REGISTER_A, 0x18);
	twiWrite(HMC5883L_ADDRESS, HMC5883L_CONFIGURATION_REGISTER_B, HMC5883L_RANGE_1_3);
	twiWrite(HMC5883L_ADDRESS, HMC5883L_MODE_REGISTER, HMC5883L_CONTINUOUS_MEASUREMENT_MODE);
	
}

void SensorStick_9DoF::sensorInit(){  
  begin();   
  delay(1000);
  
  double accZero[3] = { 0 }; // accX, accY, accZ
  double gyrZero[4] = { 0 }; // gyroX, gyroY, gyroZ,gyroTemp
  double magZero[3] = { 0 }; // magX, magY, magZ
  
  for (int i = 0; i < 100; i++) {
    IMU.receiveAll();
    accZero[0] += IMU.get(ACC,'x');
    accZero[1] += IMU.get(ACC,'y');
    accZero[2] += IMU.get(ACC,'z');
    gyrZero[0] += IMU.get(GYR,'x');
    gyrZero[1] += IMU.get(GYR,'y');
    gyrZero[2] += IMU.get(GYR,'z');
    delay(10);
  }
   accZero[0] /= 100;
   accZero[1] /= 100;
   accZero[2] /= 100;
   gyrZero[0] /= 100;
   gyrZero[1] /= 100;
   gyrZero[2] /= 100;
  
  accZero[2] -= 1;  //重力加速度の除去
  setZero(accZero,gyrZero,magZero);
}

double SensorStick_9DoF::getZero(char sensor,char axis){
  int i=0;
  switch (axis) {
    case 'x': i=0; break;
    case 'y': i=1; break;
    case 'z': i=2; break;
    case 't': i=3; break;
    default: i=axis;
  }
  
  switch (sensor) {
    case ACC: return _accZero[i]; break;
    case GYR: return _gyroZero[i]; break;
    case MAG: return _magZero[i]; break;
  }
}

int SensorStick_9DoF::setZero(char sensor,char axis,double zero[]){
 int i=0;
  switch (axis) {
    case 'x': i=0; break;
    case 'y': i=1; break;
    case 'z': i=2; break;
    case 't': i=3; break;
    default: i=axis;
  }
  
  switch (sensor) {
    case ACC: return _accZero[i] =zero[i]; break;
    case GYR: return _gyroZero[i] =zero[i]; break;
    case MAG: return _magZero[i] =zero[i]; break;
  }
}

int SensorStick_9DoF::setZero(double acc[],double gyro[],double mag[]){
  for(int i=0; i<3; i++){
    _accZero[i] = acc[i];
    _gyroZero[i] = gyro[i];
    _magZero[i] = mag[i];
  }
   _gyroZero[3] = gyro[3];
  
}


int SensorStick_9DoF::getRaw(char sensor,char axis){
  int i=0;
  switch (axis) {
    case 'x': i=0; break;
    case 'y': i=1; break;
    case 'z': i=2; break;
    case 't': i=3; break;
    default: i=axis;
  }
  
  switch (sensor) {
    case ACC: return _acc[i]; break;
    case GYR: return _gyro[i]; break;
    case MAG: return _mag[i]; break;
  }
}
double SensorStick_9DoF::get(char sensor,char axis){
  int i=0;
  switch (axis) {
    case 'x': i=0; break;
    case 'y': i=1; break;
    case 'z': i=2; break;
    case 't': i=3; break;
    default: i=axis;
  }
  
  switch (sensor) {
    case ACC: return (double)_acc[i]/accSensityivity[i]; break;
    case GYR: return (double)_gyro[i]/gyroSensityivity[i]; break;
    case MAG: return (double)_mag[i]/magSensityivity[i];; break;
  }
}

int SensorStick_9DoF::receiveGyro(){
	byte buf[8];
	if(twiRead(ITG3200_ADDRESS, ITG3200_TEMP_OUT, buf, 8) != 0){return -1;}

	_gyro[3]  = buf[0] << 8 | buf[1];
	_gyro[1] = buf[2] << 8 | buf[3]; 
	_gyro[0] = -(buf[4] << 8 | buf[5]);
	_gyro[2] = buf[6] << 8 | buf[7];
	return 0;
}

int SensorStick_9DoF::receiveAcc(){
	byte buf[6];
	if(twiRead(ADXL345_ADDRESS, ADXL345_DATAX0, buf, 6) != 0){return -1;}
	
	_acc[1] = (((int)buf[1]) << 8) | buf[0];   
	_acc[0] = (((int)buf[3]) << 8) | buf[2];
	_acc[2] = (((int)buf[5]) << 8) | buf[4];
        return 0;
}
int SensorStick_9DoF::receiveMag(){
	byte buf[6];
	if(twiRead(HMC5883L_ADDRESS, HMC5883L_DATA_REGISTER_BEGIN, buf, 6) != 0){return -1;}

	_mag[0] = ((int)buf[0] << 8) | buf[1];
	_mag[1] = ((int)buf[2] << 8) | buf[3];
	_mag[2] = ((int)buf[4] << 8) | buf[5];
  return 0;
}
int SensorStick_9DoF::receiveAll(){
    int error=0;
    error += receiveGyro();
    error += receiveAcc();
    error += receiveMag();
    return max(error,-1);
}

int SensorStick_9DoF::accRead(int acc[]){
	byte buf[6];
	if(twiRead(ADXL345_ADDRESS, ADXL345_DATAX0, buf, 6) != 0){return -1;}
	
	acc[0] = (((int)buf[1]) << 8) | buf[0];   
	acc[1] = (((int)buf[3]) << 8) | buf[2];
	acc[2] = (((int)buf[5]) << 8) | buf[4];
	return 0;
}

int SensorStick_9DoF::accRead(int* accX,int* accY,int* accZ){
	byte buf[6];
	if(twiRead(ADXL345_ADDRESS, ADXL345_DATAX0, buf, 6) != 0){return -1;}
	
	*accX = (((int)buf[1]) << 8) | buf[0];   
	*accY = (((int)buf[3]) << 8) | buf[2];
	*accZ = (((int)buf[5]) << 8) | buf[4];
	return 0;
}

int SensorStick_9DoF::gyroRead(int gyro[]){
	byte buf[6];
	if(twiRead(ITG3200_ADDRESS, ITG3200_GYRO_XOUT, buf, 6) != 0){return -1;}

	gyro[0] = buf[0] << 8 | buf[1];
	gyro[1] = buf[2] << 8 | buf[3]; 
	gyro[2] = buf[4] << 8 | buf[5];
	return 0;
}

int SensorStick_9DoF::gyroRead(int* gyroX,int* gyroY,int* gyroZ){
	byte buf[6];
	if(twiRead(ITG3200_ADDRESS, ITG3200_GYRO_XOUT, buf, 6) != 0){return -1;}

	*gyroX = buf[0] << 8 | buf[1];
	*gyroY = buf[2] << 8 | buf[3]; 
	*gyroZ = buf[4] << 8 | buf[5];
	return 0;
}

int SensorStick_9DoF::gyroRead(int gyro[],int* temp){
	byte buf[8];
	if(twiRead(ITG3200_ADDRESS, ITG3200_TEMP_OUT, buf, 8) != 0){return -1;}

	*temp    = buf[0] << 8 | buf[1];
	gyro[0] = buf[2] << 8 | buf[3]; 
	gyro[1] = buf[4] << 8 | buf[5];
	gyro[2] = buf[6] << 8 | buf[7];
	return 0;
}

int SensorStick_9DoF::gyroRead(int* gyroX,int* gyroY,int* gyroZ,int* temp){
	byte buf[8];
	if(twiRead(ITG3200_ADDRESS, ITG3200_TEMP_OUT, buf, 8) != 0){return -1;}

	*temp    = buf[0] << 8 | buf[1];
	*gyroX = buf[2] << 8 | buf[3]; 
	*gyroY = buf[4] << 8 | buf[5];
	*gyroZ = buf[6] << 8 | buf[7];
	return 0;
}

int SensorStick_9DoF::magRead(int mag[]){
	byte buf[6];
	if(twiRead(HMC5883L_ADDRESS, HMC5883L_DATA_REGISTER_BEGIN, buf, 6) != 0){return -1;}

	mag[0] = ((int)buf[0] << 8) | buf[1];
	mag[1] = ((int)buf[2] << 8) | buf[3];
	mag[2] = ((int)buf[4] << 8) | buf[5];
  return 0;
}

int SensorStick_9DoF::magRead(int* magX,int* magY,int* magZ){
	byte buf[6];
	if(twiRead(HMC5883L_ADDRESS, HMC5883L_DATA_REGISTER_BEGIN, buf, 6) != 0){return -1;}

	*magX = ((int)buf[0] << 8) | buf[1];
	*magY = ((int)buf[2] << 8) | buf[3];
	*magZ = ((int)buf[4] << 8) | buf[5];
  return 0;
}

void SensorStick_9DoF::twiWrite(byte address, byte registerAddress, byte val) {
	Wire.beginTransmission(address); // start transmission to device 
	Wire.write(registerAddress);             // send register address
	Wire.write(val);                 // send value to write
	Wire.endTransmission();         // end transmission
}

void SensorStick_9DoF::twiWrite(byte address, byte registerAddress, byte val[], byte num) {
	Wire.beginTransmission(address); // start transmission to device 
	Wire.write(registerAddress);             // send register address
	for(int i=0;i<num;i++){
	  Wire.write(val[i]);              // send value to write
	}
	Wire.endTransmission();         // end transmission
}

int SensorStick_9DoF::twiRead(byte address, byte registerAddress,byte output[], byte num) {
	Wire.beginTransmission(address); // start transmission to device 
	Wire.write(registerAddress);             // sends address to read from
	Wire.endTransmission();         // end transmission
	
	Wire.beginTransmission(address); // start transmission to device
	Wire.requestFrom(address, num);    // request n bytes from device
	
	int i = 0;
	while(Wire.available())         // device may send less than requested (abnormal)
	{ 
		if(i==num){return -1;}
		output[i] = Wire.read();    // receive a byte
		i++;
	}
	Wire.endTransmission();         // end transmission
	if(i != num){
		return -1;
	}
	return 0;
}

void SensorStick_9DoF::recvGPS(float flat, float flon, unsigned long int age) 
{
  TinyGPS gps;
  bool newData = false;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("#LA LO SA PR ");
    Serial.print(","); 
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(",");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(",");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.print(",");
    Serial.println(millis());
   
    File dataFile = SD.open("GPS.txt", FILE_WRITE);
    
    if(dataFile){
      dataFile.print("#LA LO SA PR ");
      dataFile.print(","); 
      dataFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      dataFile.print(",");
      dataFile.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      dataFile.print(",");
      dataFile.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      dataFile.print(",");
      dataFile.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
      dataFile.print(",");
      dataFile.println(millis());
      dataFile.close();
    }
    
    else {
      Serial.println("error opening datalog.txt");
    }
    
    delay(1000);
   }
}

float SensorStick_9DoF::getDt()
{
  static long lastTime=0;
  
  long nowTime = micros();
  float time = (float)(nowTime - lastTime);
  time = max(time,20);  //timeは20[us]以上
  time /= 1000000;  //[usec] => [sec]
  lastTime = nowTime;
  
  return( time );  
}
