#if defined(MAG)

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// Earth Magnetic Field appox 0.6 gauss. Must set Mag sensitivity above this but no so high that resolution is lost
// ************************************************************************************************************

#if defined(HMC5883)

void MagInit(){
  // Wait a little bit before starting
  delay(50);
  
  // Set operating mode to continuous
  Wire.beginTransmission(HMC5883L); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
}

void MagRead() {
  int x, y, z;

  // Initiate communications with compass
  Wire.beginTransmission(HMC5883L);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L, 6);    // Request 6 bytes; 2 bytes per axis
  if(Wire.available() <=6) {    // If 6 bytes available
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
  
  float sx = x * 0.92;
  float sy = y * 0.92;
  float sz = z * 0.92;
  
  float heading = atan2(sy, sx);
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;
  
  
  // Print raw values
  Serial.print("Mx=");
  Serial.print(sx);
  Serial.print(", My=");
  Serial.print(sy);
  Serial.print(", Mz=");
  Serial.print(sz);
  Serial.print(", Heading=");
  Serial.println(headingDegrees);
  
  delay(500);
}



void Mag_init() {
  int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
  bool bret=true;                // Error indicator

  delay(50);  //Wait before start
  i2c_Write(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
  // The new gain setting is effective from the second measurement and on.

  i2c_Write(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  i2c_Write(MAG_ADDRESS,HMC58X3_R_MODE, 1);
  delay(100);
  
  getADC();  //Get one sample, and discard it

  for (uint8_t i=0; i<10; i++) { //Collect 10 samples
    i2c_Write(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();   // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0]+=magADC[0];
    xyz_total[1]+=magADC[1];
    xyz_total[2]+=magADC[2];
                
    // Detect saturation.
    if (-(1<<12) >= min(magADC[0],min(magADC[1],magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  i2c_Write(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (uint8_t i=0; i<10; i++) { 
    i2c_Write(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();  // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged.
    xyz_total[0]-=magADC[0];
    xyz_total[1]-=magADC[1];
    xyz_total[2]-=magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(magADC[0],min(magADC[1],magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
  magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
  magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);

  // leave test mode
  i2c_Write(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_Write(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_Write(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
  magInit = 1;

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
} //  Mag_init()
#endif //HMC5883

#endif //MAG
