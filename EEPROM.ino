/*
void GetEepromValues(){
  //Only need EEPROM Address information
  //Put delay to make sure read is successful.
  
  #if defined(MAG)
  //Get MAG data
  MAG_MIN_X = EepromRead(MAG_MIN_X_ADDR);
  delay(5);
  MAG_MAX_X = EepromRead(MAG_MAX_X_ADDR);
  delay(5);
  MAG_MIN_Y = EepromRead(MAG_MIN_Y_ADDR);
  delay(5);
  MAG_MAX_Y = EepromRead(MAG_MAX_Y_ADDR);
  delay(5);
  MAG_MIN_Z = EepromRead(MAG_MIN_Z_ADDR);
  delay(5);
  MAG_MAX_Z = EepromRead(MAG_MAX_Z_ADDR);
  delay(5);
  #endif
  
  #if defined(ACCEL)
  ACCEL_OFFSET_X = EepromRead(ACCEL_OFFSET_X_ADDR);
  delay(5);
  ACCEL_OFFSET_Y = EepromRead(ACCEL_OFFSET_Y_ADDR);
  delay(5);  
  ACCEL_OFFSET_Z = EepromRead(ACCEL_OFFSET_Z_ADDR);
  #endif
  
}

void CheckCalibrated(){
  
  //Check to make sure MAG Configuration data NOT NULL
  
  //Check to make sure ACCEL Configuration data NOT NULL
  if (!ACCEL_OFFSET_X || !ACCEL_OFFSET_X || !ACCEL_OFFSET_X) { //ACCEL always has offset.
    //Do something.
  }
  
}
*/
