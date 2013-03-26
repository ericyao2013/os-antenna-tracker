/******************************************************************************/
/******************       I2C Generic Functions       *************************/
/******************************************************************************/

//May want to look to increasing the speed of the i2c on Arduino to 400Hz
TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz

// i2c usage - Delay should be called outside these functions if required.

void i2c_Write(byte address, byte reg, byte data){
   Wire.beginTransmission(address);
   Wire.write(reg);  
   Wire.write(data);
   Wire.endTransmission();
}

void i2c_Read(uint8_t address, uint8_t reg, uint8_t bufsize) {
  Wire.beginTransmission(address); 
  Wire.requestFrom(address, bufsize);    // request 8 bytes from ITG3200
  
  int i = 0;
  byte buff[bufsize];
  while(Wire.available())    
  { 
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
}
