#include <Wire.h>
#include "serial_master.h"

SerialMaster Master;
  
void SerialMaster::write_numbers(int numbers, int dev_number){
	Wire.beginTransmission(dev_number);
	Wire.write(numbers);
	Wire.endTransmission();
	delay(100);
}

void SerialMaster::request_data(int select_num)
{	
	int i=0;
  
	while(!Wire.available()){
		write_numbers(select_num, SLAVE_DEVICE_NUM);
		Wire.begin();
		Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
	}

	while(!Wire.available()) 
		Wire.requestFrom(SLAVE_DEVICE_NUM, ACCEL_REQUEST_BYTE);
  
	while(Wire.available()){
		tmp.accel.byte_data[i] = Wire.read();
		i++;
	} 

}
