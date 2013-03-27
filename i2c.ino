/******************************************************************************/
/******************       I2C Generic Functions       *************************/
/******************************************************************************/

//May want to look to increasing the speed of the i2c on Arduino to 400Hz
TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz

// i2c usage - Delay should be called outside these functions if required.

void i2c_write(byte address, byte reg, byte data){
   Wire.beginTransmission(address);
   Wire.write(reg);  
   Wire.write(data);
   Wire.endTransmission();
}

void i2c_read(uint8_t address, uint8_t reg, int length) {
  Wire.beginTransmission(address); 
  Wire.requestFrom(address, length);
  
  int i = 0;
  uint_8 buffer[length];
  while(Wire.available())    
  { 
    buffer[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
  
  return buffer;
}
