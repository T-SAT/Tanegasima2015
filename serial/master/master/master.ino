//com10: master
#include <Wire.h>

#define START                 '0'
#define RECEIVE               '1'

#define GPS_NUM               '1'
#define ACCEL_NUM             '2'
#define GYRO_NUM              '3'

#define SLAVE_DEVICE_NUM       2

typedef struct {
  float f_data1;
  float f_data2;
  float f_data3;
} accel;

typedef union {
  accel f_data;
  uint8_t data[sizeof(float)*3];
} test_u;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  request_data(ACCEL_NUM);
}

void request_data(char select_num)
{
  test_u test1;
  int i=0;
  byte check = '0';
  
  while(check != RECEIVE){
    Serial.print(select_num);
    delay(100);
    check = Serial.read();
  }
  
  while(check != START)
    check = Serial.read();
    
  while(!Wire.available()){
    Wire.requestFrom(SLAVE_DEVICE_NUM, sizeof(float)*3);
    delay(100);
  }
  
  while(Wire.available()){
    test1.data[i] = Wire.read();
    i++;
  }
  
  //Serial.print("test1.f_data.f_data1 = ");
  Serial.println(test1.f_data.f_data1);
}
 
  
