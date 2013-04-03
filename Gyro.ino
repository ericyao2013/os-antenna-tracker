void GYRO_init() {
  
  Serial.println("Initialising Gyro");
  delay(1000);
  gyro.reset();
  gyro.init(ITG3200_ADDR_AD0_LOW);
  i2cWrite(0x68,0x16,0x1A); // this puts your gyro at +-2000deg/sec and 98Hz Low pass filter
  i2cWrite(0x68,0x15,0x09); // this sets your gyro at 100Hz sample rate
  Serial.println("done.");  
   
}

void GYRO_setLevel() {

  Serial.print("Zero Calibrating Gyro");
  delay(1000);
  Serial.print("Do not move board");
  gyro.zeroCalibrate(2500,2);
  Serial.println("Gyro Done.");
 
}

void GYRO_setLevel() {
  
  // Gyro XYZ array
  float xyz[3];
  
  //Number of readings to average
  int number = 2500;
  
  Serial.print("Zero Calibrating Gyro");
  delay(1000);
  Serial.print("Do not move board");
  delay(1000);

  
  // Give the Gyro internal LPF time to warm up
  // Only do 1/5th of the number of readings
  for (j = 0; j < number/5; j ++){
    gyro.readGyro(xyz);
    delay(3);
  }
  
  // Take 2500 readings and average them.
  for (j = 0; j < number; j ++){
    gyro.readGyro(xyz);
    
    int gyroSumX += x;
    int gyroSumY += y;
    int gyroSumZ += z;
    
    delay(3);
  }
  
  float offsetX = gyroSumX / number;
  float offsetY = gyroSumY / number;
  float offsetZ = gyroSumZ / number;
  
  delay(1000);
  Serial.println("Gyro Calibration Done.");
  
  // Save Calibration data
  eeprom_write(GYRO_CAL_FLAG_ADDR);
  eeprom_write(GYRO_ZERO_X_ADDR);
  eeprom_write(GYRO_ZERO_Y_ADDR);
  eeprom_write(GYRO_ZERO_Z_ADDR);
  
  delay(2000);
}

void GYRO_read() {
  
 float temperature;
  
  // Read GYRO temperature
  gyro.readTemp(&temperature);
  // Read GYRO Values
  
}
