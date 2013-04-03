void MAG_init(){

  mag.SetScale(1.3);
  mag.SetMeasurementMode(Measurement_Continuous);
  
}

void MAG_calibrate(void) {

  delay(5000);
  Serial.println("MAGNETOMETER CALIBRATION");
  delay(1000);
  Serial.println("To calibrate rotate the board around each axis");
  delay(1000);
  Serial.println("Calibration will run for 30 seconds");
  delay(2000);
  Serial.println();
  // Countdown for Calibrate
  for (int cc = 1; cc < 6; cc++){
    Serial.print("Calibration Starting in ");Serial.print(6-cc);Serial.println(" seconds");
    delay(1000);
  }
  
  Serial.println("Calibration Started");    
    
    // Reads the Mag Raw Axis
    for(int x= millis(); x< millis() + 30000 ; x = +millis())
    {
      
      Serial.println(x);
      magRaw = mag.ReadRawAxis();
      magScaled = mag.ReadScaledAxis();
  
      
      magMinMax(magScaled.XAxis, magScaled.YAxis);
      magCalibrationOutput();
      delay(10);
    }
    
    magCalcScaleFactor_Offset();
    magCalibrationComplete();
    storeMagCalibration();    //save the calculated x & y offset and scale factor
    Serial.println();
    Serial.println("*************************************************************");
    delay(1000);
    Serial.println("Magnetometer has been calibrated");
    delay(1000);
    Serial.println("Please comment out CALIBRATE_MAG and reload the board software");
    delay(5000);  
  
}

// Get Min and Max MAG readings for calibration
//*******************************************************
void magMinMax(int xRaw, int yRaw)
{
  
  int maxReading = 800; // You can change this if you want.
  
  // If the readings are too high, drop them.
  if(xRaw > maxReading || xRaw<-maxReading || yRaw>maxReading || yRaw<-maxReading)
    return;
   
  if(xRaw > xMax)
    xMax = xRaw;

  if(xRaw < xMin)
    xMin = xRaw;
 
  if(yRaw > yMax)
    yMax = yRaw;
 
  if(yRaw < yMin)
    yMin = yRaw;
   
}

void magCalcScaleFactor_Offset()
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



void magCalibrationOutput()
{
  //raw magnetometer x&y values
  Serial.print("RawMag x=");
  Serial.print(magRaw.XAxis);
  Serial.print(",y=");
  Serial.print(magRaw.YAxis);
  Serial.print(" ");
  //mag values scled with gause gain setting 
  Serial.print("ScaledMag x=");
  Serial.print(magScaled.XAxis);
  Serial.print(",y=");
  Serial.print(magScaled.YAxis);
  Serial.print(" "); 
  //current max min value as calibration proceeds
  Serial.print(" xMax=");
  Serial.print(xMax);
  Serial.print(",yMax=");
  Serial.print(yMax);
  Serial.print(", xMin=");
  Serial.print(xMin);
  Serial.print(", yMin=");
  Serial.println(yMin);

}

void magCalibrationComplete()
{
  //min/max values found
  Serial.print("MaxMin xMax=");
  Serial.print(xMax);
  Serial.print(",yMax=");
  Serial.print(yMax);
  Serial.print(", xMin=");
  Serial.print(xMin);
  Serial.print(", yMin=");
  Serial.print(yMin);
  //calculated offset and scale factors
  Serial.print("xScale=");
  Serial.print(xScaleFactor);
  Serial.print(",yScale=");
  Serial.print(yScaleFactor);
  Serial.print(", XOffset=");
  Serial.print(compassXOffset);
  Serial.print(", YOffset=");
  Serial.print(compassYOffset); 

}

/*************************************************************************/
//save calibration data to eeprom
void storeMagCalibration(){
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


// Read calibration data from eeprom
void readMagCalibration(){
 
  int compassXOffset = (EEPROM.read(1) * 256) + EEPROM.read(0);
  int compassYOffset = (EEPROM.read(3) * 256) + EEPROM.read(2);
  int xScaleFactor = (EEPROM.read(5) * 256) + EEPROM.read(4);
  int yScaleFactor = (EEPROM.read(7) * 256) + EEPROM.read(6);
 
}
