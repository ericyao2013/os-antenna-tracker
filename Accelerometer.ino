// Initialise BMA180
void ACCEL_init(){
  
  accel.setAddress(BMA180_ADDRESS_SDO_LOW);
  accel.SoftReset();
  accel.enableWrite();
  accel.SetFilter(accel.F10HZ);
  accel.setGSensitivty(accel.G2);
  accel.SetSMPSkip();
  accel.SetISRMode();
  accel.disableWrite();
  delay(100);
  
}

// 6 Point calibration to get each axis range.
void ACCEL_calibrate(){
  
  // Accel XYZ array
  int axyz[3];
  
  //Number of readings to average
  int number = 2500;
  
  // Offset Values
  float averageX = 0;
  float averageY = 0;
  float averageZ = 0;

  Serial.println("Accelerometer 6 point calibration");
  delay(1000);
  Serial.println("There is no order of calibration - just keep flipping the board until all axis' are calibrated");
  delay(1000);
  Serial.println("After each axis is calibrated, move the board to calibrate the next axis");
  delay(1000);
  Serial.println("If you bump the board while calibrating, let it be recalibrated");
  delay(1000);  
  Serial.println("Calibration will exit once all sides are calibrated");
  delay(1000);
  Serial.println("IMPORTANT: Do not bump or move the board while calibration is running");
  delay(1000);
  Serial.println("Lay board flat now....");  
  
  // Give the Accel internal LPF time to warm up
  // Only do 1/5th of the number of readings
  for (int j = 0; j < number / 5; j ++){
    accel.readAccel(axyz);
    delay(3);
    
  }
  
  // Calibration variables
  float xMin = 0;
  float xMax = 0;
  float yMin = 0;
  float yMax = 0;
  float zMin = 0;
  float zMax = 0;
  
   while (xMin == 0 || xMax == 0 || yMin == 0 || yMax == 0 || zMin == 0 || zMax == 0){
    
    long accelSumX = 0;
    long accelSumY = 0;
    long accelSumZ = 0;
    
    for (int i = 0; i < 10; i ++){
      Serial.print("Calibrating Axis in ");Serial.println(10-i);
      delay(1000);
    }
    Serial.println("Calibrating......");
    
    // Take 2500 readings and average them.
    for (int j = 0; j < number; j ++){
      accel.readAccel(axyz);
      
      accelSumX += axyz[0];
      accelSumY += axyz[1];
      accelSumZ += axyz[2];
      
      delay(3);
    }
    
    // X axis is the side thats experiencing +1G or -1G
    if(abs(accelSumX) > abs(accelSumY) && abs(accelSumX) > abs(accelSumZ)){
      averageX = accelSumX / number;
      // Sensing negative value
      if (averageX < 0) {
        Serial.print("Min X: ");Serial.println(averageX);
        xMin = averageX;
      }
        else{
        Serial.print("Max X: ");Serial.println(averageX);
        xMax = averageX;
        }
    }
    // Y axis is the side thats experiencing +1G or -1G  
    else if(abs(accelSumY) > abs(accelSumX) && abs(accelSumY) > abs(accelSumZ)){
    averageY = accelSumY / number;
    // if sensing negative value
    if (averageY < 0) {
        Serial.print("Min Y: ");Serial.println(averageY);
        yMin = averageY;
    }
      else{
        Serial.print("Max Y: ");Serial.println(averageY);
        yMax = averageY;
      }
    }
    // Z axis is the side thats experiencing +1G or -1G  
    else if(abs(accelSumZ) > abs(accelSumX) && abs(accelSumZ) > abs(accelSumY)){
    averageZ = accelSumZ / number;
    // if sensing negative value
    if (averageZ < 0) {
        Serial.print("Min Z: ");Serial.println(averageZ);
        zMin = averageZ;
    }
      else{
        Serial.print("Max Z: ");Serial.println(averageZ);
        zMax = averageZ;
      }   
    }
  }
  
  // We need the total range to calculate the offset and scale
  int acc_range_x = (abs(xMin) + abs(xMax));
  int acc_range_y = (abs(yMin) + abs(yMax));
  int acc_range_z = (abs(zMin) + abs(zMax));
  
  // Scale over 1G so divide by 2 as the total range is 2G
  acc_scale_x = acc_range_x / 2;
  acc_scale_y = acc_range_y / 2;
  acc_scale_z = acc_range_z / 2;
  
  acc_off_x =  xMax - (acc_range_x / 2);
  acc_off_y =  yMax - (acc_range_y / 2);
  acc_off_z =  zMax - (acc_range_z / 2);

  // Save calibration data
  //eeprom_write(ACCEL_CAL_FLAG_ADDR, 1);
  
  Serial.println("Accel Calibration Done.");
  delay(1000);
  //Save calibration data to EEPROM
  Serial.println("Saving Calibration data");
  Serial.print("Accel Offset X: ");Serial.print(acc_off_x);Serial.print("  ");
  Serial.print("Accel Offset Y: ");Serial.print(acc_off_y);Serial.print("  ");
  Serial.print("Accel Offset Z: ");Serial.println(acc_off_z);
  Serial.print("Accel Scale X: ");Serial.print(acc_scale_x);Serial.print("  ");
  Serial.print("Accel Scale Y: ");Serial.print(acc_scale_y);Serial.print("  ");
  Serial.print("Accel Scale Z: ");Serial.println(acc_scale_z);

}

void ACCEL_calibrateQ(){
 
  // Accel has a max range of 8192 per G (or 2 x 8191).
  float accelRange = 8191 / 2;
  
  long accelSumX = 0;
  long accelSumY = 0;
  long accelSumZ = 0;  
  
  // Accel XYZ array
  int axyz[3];
  
  //Number of readings to average
  int number = 2500;
  
  delay(1000);
  Serial.println("Accel Calibration (quick) - DO NOT MOVE BOARD");
  
  // Take 2500 readings and average them.
  for (int j = 0; j < number; j ++){
    accel.readAccel(axyz);
    
    accelSumX += axyz[0];
    accelSumY += axyz[1];
    accelSumZ += axyz[2];
    
    delay(3);
  }
  
  // Scale over 1G so divide by 2 as the total range is 2G
  acc_scale_x = accelRange;
  acc_scale_y = accelRange;
  acc_scale_z = accelRange;
  
  acc_off_x =  accelSumX / number;
  acc_off_y =  accelSumY / number;
  acc_off_z =  (abs(accelSumZ) / number) - accelRange;
  
  Serial.println("Accel Calibration Done.");
  delay(1000);
  Serial.println("Saving Calibration data");
  //Save calibration data to EEPROM
  Serial.print("Accel Offset X: ");Serial.print(acc_off_x);Serial.print("  ");
  Serial.print("Accel Offset Y: ");Serial.print(acc_off_y);Serial.print("  ");
  Serial.print("Accel Offset Z: ");Serial.println(acc_off_z);
  Serial.print("Accel Scale X: ");Serial.print(acc_scale_x);Serial.print("  ");
  Serial.print("Accel Scale Y: ");Serial.print(acc_scale_y);Serial.print("  ");
  Serial.print("Accel Scale Z: ");Serial.println(acc_scale_z);
  
}
