//initializes the gyroscope
void initGyro () {
  /*****************************************
  * ITG 3200
  * power management set to:
  * clock select = internal oscillator
  *     no reset, no sleep mode
  *   no standby mode
  * sample rate to = 3Hz
  * parameter to +/- 2000 degrees/sec
  * low pass filter = 5Hz
  * no interrupt
  ******************************************/
 
  writeTo2 (GYRO_ADDR, PWR_MGM, 0x00);
  writeTo2 (GYRO_ADDR, SMPLRT_DIV, 0xFF); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo2 (GYRO_ADDR, DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo2 (GYRO_ADDR, INT_CFG, 0x00);
}

void getGyroscopeData (int *xG, int *yG, int *zG) {
  /**************************************
  Gyro ITG-3200 I2C
  registers:
  x axis MSB = 1D, x axis LSB = 1E
  y axis MSB = 1F, y axis LSB = 20
  z axis MSB = 21, z axis LSB = 22
  *************************************/
 
  int regAddress = 0x1D;
  int x, y, z;
  byte buff[TO_READ];
  char str[50]; // 50 should be enough to store all the data from the
 
  readFrom2(GYRO_ADDR, regAddress, TO_READ, buff); //read the gyro data from the ITG3200
 
  x = (buff[0] << 8) | buff[1];
  y = (buff[2] << 8) | buff[3];
  z = (buff[4] << 8) | buff[5];
  //we send the x y z values as a string to the serial port
  *xG = x;
  *yG = y;
  *zG = z;
}
 
void readAccel(int *xA, int *yA, int *zA) {
  unsigned int time;
  
  uint8_t howManyBytesToRead = 6;
  readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345
 
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int x = (((int)_buff[1]) << 8) | _buff[0];   
  int y = (((int)_buff[3]) << 8) | _buff[2];
  int z = (((int)_buff[5]) << 8) | _buff[4];
  *xA = x;
  *yA = y;
  *zA = z;
}
 
void writeTo(byte address, byte val) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}
 
// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission
 
  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from device
 
  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff[i] = Wire.read();    // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}

//---------------- Functions
//Writes val to address register on device
void writeTo2 (int device, byte address, byte val) {
   Wire.beginTransmission (device); //start transmission to device
   Wire.write (address);	    // send register address
   Wire.write(val);         // send value to write
   Wire.endTransmission (); //end transmission
}
 
 
//reads num bytes starting from address register on device in to buff array
void readFrom2 (int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission (device); //start transmission to device
  Wire.write(address);             //sends address to read from
  Wire.endTransmission ();         //end transmission
 
  Wire.beginTransmission (device); //start transmission to device
  Wire.requestFrom (device, num);  // request 6 bytes from device
 
  int i = 0;
  while (Wire.available ()) {      //device may send less than requested (abnormal)
    buff[i] = Wire.read ();     //receive a byte
    i++;
  }
  Wire.endTransmission ();         //end transmission
}

