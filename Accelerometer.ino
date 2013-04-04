// Acceleromters detect acceleration but they detect it in the opposite direction to the acceleration.
// For example if you were accelerating towards the Y positive, the acceleration would be a Y negative value.

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
  int xyz[3];
  
  //Number of readings to average
  int number = 2500;
  
  // Offset Values
  float averageX = 0;
  float averageY = 0;
  float averageZ = 0;

  Serial.print("Accelerometer 6 point calibration");
  delay(1000);
  Serial.print("There is no order of calibration - just keep flipping the board until all axis' are calibrated");
  delay(1000);
  Serial.print("After each axis is calibrated, move the board to calibrate the next axis");
  delay(1000);
  Serial.print("If you bump the board while calibrating, let it be recalibrated");
  delay(1000);  
  Serial.print("Calibration will exit once all sides are calibrated");
  delay(1000);
  Serial.print("IMPORTANT: Do not bump or move the board while calibration is running");
  delay(1000);
  Serial.print("Lay board flat now....");  
  
  // Give the Accel internal LPF time to warm up
  // Only do 1/5th of the number of readings
  for (int j = 0; j < number/5; j ++){
    accel.readAccel(xyz);
    delay(3);
    
  }
  
    // Calibration flag variables
  int XMincalibrated = 0;
  int XMaxcalibrated = 0;
  int YMincalibrated = 0;
  int YMaxcalibrated = 0;
  int ZMincalibrated = 0;
  int ZMaxcalibrated = 0;
  
   while (XMincalibrated == 0 || XMaxcalibrated == 0 || YMincalibrated == 0 || YMaxcalibrated == 0 || ZMincalibrated == 0 || ZMaxcalibrated == 0){
    
    long accelSumX = 0;
    long accelSumY = 0;
    long accelSumZ = 0;
    
    for (int i = 0; i < 10; i ++){
      Serial.print("Calibrating Axis in ");Serial.println(10-i);
      delay(1000);
      Serial.println("Calibrating");
    }
    
    // Take 2500 readings and average them.
    for (int j = 0; j < 2500; j ++){
      accel.readAccel(xyz);
      
      accelSumX += xyz[0];
      accelSumY += xyz[1];
      accelSumZ += xyz[2];
      
      delay(3);
    }
    
    // X axis is the side thats experiencing +1G or -1G
    if(abs(accelSumX) > abs(accelSumY) && abs(accelSumX) > abs(accelSumZ)){
      averageX = accelSumX / number;
      // Sensing negative value
      if (averageX < 0) {
        Serial.print("Min X: ");Serial.println(averageX);
        eeprom_write(ACCEL_MIN_X_ADDR, averageX);
        XMincalibrated = 1;
      }
        else{
        Serial.print("Max X: ");Serial.println(averageX);
        eeprom_write(ACCEL_MAX_X_ADDR, averageX);
        XMaxcalibrated = 1;
        }
    }
    // Y axis is the side thats experiencing +1G or -1G  
    else if(abs(accelSumY) > abs(accelSumX) && abs(accelSumY) > abs(accelSumZ)){
    averageY = accelSumY / number;
    // if sensing negative value
    if (averageY < 0) {
        Serial.print("Min Y: ");Serial.println(averageY);
        eeprom_write(ACCEL_MIN_Y_ADDR, averageY);
        YMincalibrated = 1;
    }
      else{
        Serial.print("Max Y: ");Serial.println(averageY);
        YMaxcalibrated = 1;
        delay(1000);
      }
    }
    // Z axis is the side thats experiencing +1G or -1G  
    else if(abs(accelSumZ) > abs(accelSumX) && abs(accelSumZ) > abs(accelSumY)){
    averageZ = accelSumZ / number;
    // if sensing negative value
    if (averageZ < 0) {
        Serial.print("Min Z: ");Serial.println(averageZ);
        ZMincalibrated = 1;
    }
      else{
        Serial.print("Max Z: ");Serial.println(averageZ);
        ZMaxcalibrated = 1;
      }   
    }
  }
    
  
  Serial.println("Accel Calibration Done.");
  delay(1000);  
  // Save calibration data
  eeprom_write(ACCEL_CAL_FLAG_ADDR, 1);

  eeprom_write(ACCEL_MAX_X_ADDR 18
  eeprom_write(ACCEL_MIN_Y_ADDR 20
  eeprom_write(ACCEL_MAX_Y_ADDR 22
  eeprom_write(ACCEL_MIN_Z_ADDR 24
  eeprom_write(ACCEL_MAX_Z_ADDR 26
  
  delay(2000);
  
}


