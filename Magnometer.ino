#if defined(MAG)

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// Earth Magnetic Field appox 0.6 gauss. Must set Mag sensitivity above this but no so high that resolution is lost
// ************************************************************************************************************

#if defined(HMC5883)

void HMC5883_init(){
  // Wait 50ms bit before starting
  delay(50);
  
  // Set operating mode to continuous
  i2c_Write(HMC5883_ADDRESS, HMC5883_R_MODE, 0x00);
  
  // Set scale to 1.3 Gauss
  i2c_Write(HMC5883_ADDRESS, HMC5883_R_CONFB, HMC5883_GAIN_13);
 
}

void HMC5883_read() {
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
  
  // C
  float sx = x * HMC5883_GAIN_13_SCALE;
  float sy = y * HMC5883_GAIN_13_SCALE;
  float sz = z * HMC5883_GAIN_13_SCALE;
  
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



void calibrate(void) {

  delay(5000);
  Serial.println("MAGNETOMETER CALIBRATION");
  delay(1000);
  Serial.println("To calibrate rotate the board around each axis");
  delay(1000);
  Serial.println("Calibration will run for 30 seconds");
  delay(2000);
  Serial.println();
  for (int cc = 1; cc < 6; cc++){
    Serial.print("Calibration Starting in ");Serial.print(6-cc);Serial.println(" seconds");
    delay(1000);
  }
  Serial.println("Calibration Started");
           
        //delay(5000);  //wait a couple seconds for operator to get arrow ponted north
        //delayMillis = millis() + 2000;
        for(int x=millis(); x<millis()+30000 ; x = +millis())  //continualy read the raw axis data while waiting for operator to slowly rotate compass to next position
        {
          magraw = compass.ReadRawAxis();
          scaled = compass.ReadScaledAxis();               
          compassMaxMin(scaled.XAxis,scaled.YAxis);  //look for max and min values
          calibrationOutput();  //output current data as calibration proceeds
          delay(20);
        }
       
  calcScaleFactor_Offset();
  calibrationComplete();  //output the resulting values from calibration   
  storeCal();    //save the calculated x & y offset and scale factor
  Serial.println();
  Serial.println("*************************************************************");
  Serial.println("Magnetometer has been calibrated");
  delay(1000);
  Serial.println();
  
 
}

    //used calibrating hmc5883l compass
    //*******************************************************
    void compassMaxMin(int xRaw, int yRaw)
    {
      if(xRaw>800 || xRaw<-800 || yRaw>800 || yRaw<-800)
        return; //prevent extraneous high readings from messing with calibration
       
      if(xRaw > xMax)
        xMax = xRaw;

      if(xRaw < xMin)
        xMin = xRaw;
     
      if(yRaw > yMax)
        yMax = yRaw;
     
      if(yRaw < yMin)
        yMin = yRaw;
       
    }

    //*******************************************************************
    //calculate the x&y scale factor and offsets for hmc5883l
    //compensates for objects around sensor distorting magnetometer values
    void calcScaleFactor_Offset()
    {
      xScaleFactor = (yMax - yMin)/(xMax - xMin);
      if(xScaleFactor < 1)
        xScaleFactor = 1;

      yScaleFactor = (xMax - xMin)/(yMax - yMin);
      if(yScaleFactor < 1)
        yScaleFactor = 1;

      compassXOffset = ((xMax - xMin)/2 - xMax) * xScaleFactor;
      compassYOffset = ((yMax - yMin)/2 - yMax) * yScaleFactor;
       
    }

    /*************************************************************************/ 
    //save calibration data to eeprom
    void storeCal(){
      //write x & y scale factors and offsets to eeprom 
        EEPROM.write(0,lowByte(compassXOffset));   
        EEPROM.write(1,highByte(compassXOffset));
       
        EEPROM.write(2,lowByte(compassYOffset));   
        EEPROM.write(3,highByte(compassYOffset));
       
        EEPROM.write(4,lowByte(xScaleFactor));   
        EEPROM.write(5,highByte(xScaleFactor));
       
        EEPROM.write(6,lowByte(yScaleFactor));   
        EEPROM.write(7,highByte(yScaleFactor));
         
    }

    /*************************************************************************/ 
    //read calibration data from eeprom
    void readCal(){
     
      compassXOffset = (EEPROM.read(1) * 256) + EEPROM.read(0);
      compassYOffset = (EEPROM.read(3) * 256) + EEPROM.read(2);
      xScaleFactor = (EEPROM.read(5) * 256) + EEPROM.read(4);
      yScaleFactor = (EEPROM.read(7) * 256) + EEPROM.read(6);
     
    }
