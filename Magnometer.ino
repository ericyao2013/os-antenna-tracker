// HMC5883L initialise
void MAG_init(){

  mag.init(false); // Don't set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  //mag.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  mag.setMode(0);
  delay(10);
  mag.setDOR(B110);
  
  
  //mag.SetScale(1.3);
  //mag.SetMeasurementMode(Measurement_Continuous);  
  
}

void MAG_calibrate() {
  
  Serial.println();
  Serial.println("Calibrating Magnetometer");
  delay(1000);
  Serial.println("Move the board around all its axis'");
  Serial.println("Calibration will run for approx 20 seconds");
  delay(1000);
    // Countdown to calibration
    for (int cc = 0; cc < 5; cc++){
      Serial.print("Calibration Starting in ");Serial.print(5-cc);Serial.println(" seconds");
      delay(1000);
    }
  
  // Array for Mag values
  int mxyz[3];
  const int samples = 2500;
  int xMax = 0, xMin = 0, yMax = 0, yMin = 0, zMax = 0, zMin = 0;
  
  // Calibration part
  for (int i = 0; i < samples; i ++){
    delay(5);
    mag.getRaw(&mxyz[0], &mxyz[1], &mxyz[2]);
    
    int xRaw = mxyz[0];
    int yRaw = mxyz[1];
    int zRaw = mxyz[2];
    
    // Get the Min and Max Values
    // Throw away any values that are stupidly high - error measurements
    if(xRaw < 800 && xRaw > -800 && yRaw < 800 && yRaw > -800 && zRaw < 800 && zRaw > -800){
    // X Axis
      if(xRaw > xMax) xMax = xRaw;
      if(xRaw < xMin) xMin = xRaw;
    // Y Axis
      if(yRaw > yMax) yMax = yRaw;
      if(yRaw < yMin) yMin = yRaw;
    // Z Axis
      if(zRaw > zMax) zMax = zRaw;
      if(zRaw < zMin) zMin = zRaw;
      
      Serial.print("Max X: ");Serial.print(xMax);Serial.print(" ");Serial.print("Min X: ");Serial.print(xMin);Serial.print(" ");
      Serial.print("Max Y: ");Serial.print(yMax);Serial.print(" ");Serial.print("Min Y: ");Serial.print(yMin);Serial.print(" ");
      Serial.print("Max Z: ");Serial.print(zMax);Serial.print(" ");Serial.print("Min Z: ");Serial.println(zMin);
    }
  }
  
  //(counts_per_milligauss[gain]*(HMC58X3_X_SELF_TEST_GAUSS*2))/(xyz_total[0]/n_samples);
  
  magn_scale_x = (abs(xMax) + abs(xMin));
  magn_scale_y = (abs(yMax) + abs(yMin));
  magn_scale_z = (abs(zMax) + abs(zMin));
  
  // Halve the total Mag value for each axis and remove the negative from it.
  // The value that results will either be a positive or negative and this will be the value of the mag when flat and level.
  magn_off_x =  xMax - (magn_scale_x / 2);
  magn_off_y =  yMax - (magn_scale_y / 2);
  magn_off_z =  zMax - (magn_scale_z / 2);

  Serial.println("Magnetometer Calibration - Finished");
  delay(1000);
  Serial.println("Printing Results");
  delay(1000);
  Serial.print("Min X: ");Serial.print(xMin);Serial.print("  ");Serial.print("Max X: ");Serial.print(xMax);Serial.print("  ");
  Serial.print("Min Y: ");Serial.print(yMin);Serial.print("  ");Serial.print("Max Y: ");Serial.print(yMax);Serial.print("  ");
  Serial.print("Min Z: ");Serial.print(zMin);Serial.print("  ");Serial.print("Max Z: ");Serial.println(zMax);
  delay(1000);
  Serial.print("Scale X: ");Serial.print(magn_scale_x);Serial.print("  ");Serial.print("Scale Y: ");Serial.print(magn_scale_y);Serial.print("  ");Serial.print("Scale Z: ");Serial.println(magn_scale_z);
  delay(1000);  
  Serial.print("Offset X: ");Serial.print(magn_off_x);Serial.print("  ");Serial.print("Offset Y: ");Serial.print(magn_off_y);Serial.print("  ");Serial.print("Offset Z: ");Serial.println(magn_off_z);
  delay(6000);
}
