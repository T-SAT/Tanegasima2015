//com12: slave
#include <Wire.h>
#include "SensorStick_9DoF.h"
#include "SerialSlave.h"

#define START               '0'
#define RECEIVE             '1'

#define DATA_1              '1'
#define DATA_2              '2'
#define DATA_3              '3'

#define SLAVE_DEVICE_NUM     2
typedef struct {
  float f_data1;
  float f_data2;
  float f_data3;
} accel;

typedef union {
  accel f_data;
  uint8_t data[sizeof(float)*3];
} test_u;

test_u tmp;
double gyroX, gyroY, gyroZ;
double accXval, accYval, accZval;

sensorData test;

void setup()
{  
  Serial.begin(9600);
  Serial.setintr(receive_data);
  IMU.sensorInit();
}

void loop()
{
  IMU.receiveAcc();
  IMU.receiveGyro();
  gyroX   = IMU.get(GYR,'x') - IMU.getZero(GYR,'x');  //オフセットぶんを差し引く
  gyroY   = IMU.get(GYR,'y') - IMU.getZero(GYR,'y');
  gyroZ   = IMU.get(GYR,'z') - IMU.getZero(GYR,'z');
  accXval = IMU.get(ACC,'x') - IMU.getZero(ACC,'x');
  accYval = IMU.get(ACC,'y') - IMU.getZero(ACC,'y');  //オフセットぶんを差し引く
  accZval = IMU.get(ACC,'z') - IMU.getZero(ACC,'z');  //オフセットぶんを差し引く
  test.accel.float_data.xA = accXval;
  test.accel.float_data.yA = accYval;
  test.accel.float_data.zA = accZval;
  interrupts();
  delay(100);
}

void send_Accel(void)
{
  Wire.write(test.accel.byte_data, sizeof(test.accel.byte_data));
  Wire.begin();
  noInterrupts();
}

void send_Gyro(void)
{
  Serial.print("tmp.f_data = ");
  Serial.println(tmp.f_data.f_data2);
  Wire.write(tmp.data, sizeof(tmp.data));
  delay(1000);
}

void send_GPS(void)
{
  Serial.print("tmp.f_data = ");
  Serial.println(test.gps.gps_data.flat);
  Wire.write(test.gps.byte_data, sizeof(test.gps.byte_data));
  delay(1000);
}

void Test4(void)
{
  Serial.println("error!");
}

void receive_data(ring_buffer *buf)
{
  char check;
  
  noInterrupts();
  check = Serial.read();
  interrupts();
  Serial.print(RECEIVE);
  Wire.begin(SLAVE_DEVICE_NUM);
  
  if(check == GPS_NUM || check == ACCEL_NUM || check == GYRO_NUM){
    switch(check){
    case GPS_NUM:
      Wire.onRequest(send_GPS);
      break;
  
    case ACCEL_NUM:
      Wire.onRequest(send_Accel);
      Serial.print(START);
      break;
    
    case GYRO_NUM:
      Wire.onRequest(send_Gyro);
      break;
    
    default :
      Wire.onRequest(Test4);
      break; 
    }
  }
  
}

