void GYRO_init() {
  gyro.init(ITG3200_ADDR_AD0_LOW);
  //i2c_write(0x68,0x16,0x1A); // this puts your gyro at +-2000deg/sec and 98Hz Low pass filter
  //i2c_write(0x68,0x15,0x09); // this sets your gyro at 100Hz sample rate
   
}

/*******************************************************************************************************/
void GYRO_calibrate() {
  
  Serial.println();
  delay(5000);
  Serial.println("Calibrating Gyro - DO NOT MOVE BOARD");
  delay(2000);
  const int samples = 1000.0;
  int gxyz[3];
  long gxTot = 0, gyTot = 0, gzTot = 0;
  
  for (int i = 0; i < samples; i++){
    delay(5);
    gyro.readGyroRaw(&gxyz[0], &gxyz[1], &gxyz[2]);
    
    gxTot += gxyz[0];
    gyTot += gxyz[1];
    gzTot += gxyz[2];
    
    //Serial.print("Xtot: ");Serial.print(gxTot);Serial.print(" ");Serial.print("Ytot: ");Serial.print(gyTot);Serial.print(" ");Serial.print("Ztot: ");Serial.println(gzTot);
  }
  
  gyro_off_x = gxTot / samples;
  gyro_off_y = gyTot / samples;
  gyro_off_z = gzTot / samples;

  Serial.println("Gyro Calibration Finished");
  delay(1000);
  Serial.print("Gyro offset X: ");Serial.print(gyro_off_x);Serial.print("  ");
  Serial.print("Gyro offset Y: ");Serial.print(gyro_off_y);Serial.print("  ");
  Serial.print("Gyro offset Z: ");Serial.println(gyro_off_z);
  delay(3000);  
}
