/******************************************************************************/
/******************       I2C Generic Functions       *************************/
/******************************************************************************/

//May want to look to increasing the speed of the i2c on Arduino to 400Hz
//TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz

// i2c usage - Delay should be called outside these functions if required.
/*
void i2c_write(byte address, byte reg, byte data){
   Wire.beginTransmission(address);
   Wire.write(reg);  
   Wire.write(data);
   Wire.endTransmission();
}

byte * i2c_read(byte address, byte reg, int length) {
  //Create the Array that will hold the data
  byte buffer[length];
  
  Wire.beginTransmission(address); 
  Wire.requestFrom(address, length);

  //if(Wire.available() !6) return; - this is meant to be the error part.
  
  for(int i = 0; i < length; i++)
    buffer[i] = Wire.read();
  Wire.endTransmission();  
  return buffer;

}*/
