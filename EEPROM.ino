// For simplicities sake all values are two bytes
// ATMega 2650 has 4096 bytes of EEPROM = 2048 values at 2 bytes per value.
// ATmega 328P has 1024 bytes of EEPROM = 512 values at 2 bytes per value.
// Both of the above have enough EEPROM to be lazy about storage sizes.

// Read int (2 bytes) from EEPROM
int eeprom_read(int address) {
    return ((EEPROM.read(address + 1) << 0) & 0xFF) + ((EEPROM.read(address) << 8) & 0xFF00);
}

// Write variable to EEPROM
void eeprom_write(int address, int var) {
    EEPROM.write(address, highByte(var));
    EEPROM.write(address + 1, lowByte(var));
}

void GetEepromValues(int calData[]){
    
    Serial.println("Loading from EEPROM");
    delay(5000);

    // Read Magnetometer calibration data
    calData[0] = eeprom_read(CAL_FLAG_ADDR);    
    calData[1] = eeprom_read(MAG_MIN_X_ADDR);
    calData[2] = eeprom_read(MAG_MAX_X_ADDR);
    calData[3] = eeprom_read(MAG_MIN_Y_ADDR);
    calData[4] = eeprom_read(MAG_MAX_Y_ADDR);
    calData[5] = eeprom_read(MAG_MIN_Z_ADDR);
    calData[6] = eeprom_read(MAG_MAX_Z_ADDR);

    // Read Acceleromter calibration data
    calData[7] = eeprom_read(ACCEL_MIN_X_ADDR);
    calData[8] = eeprom_read(ACCEL_MAX_X_ADDR);
    calData[9] = eeprom_read(ACCEL_MIN_Y_ADDR);
    calData[10] = eeprom_read(ACCEL_MAX_Y_ADDR);
    calData[11] = eeprom_read(ACCEL_MIN_Z_ADDR);
    calData[12] = eeprom_read(ACCEL_MAX_Z_ADDR);    
    
    // Read Gyro calibration data
    calData[13] = eeprom_read(GYRO_ZERO_X_ADDR);
    calData[14] = eeprom_read(GYRO_ZERO_Y_ADDR);
    calData[15] = eeprom_read(GYRO_ZERO_Z_ADDR);

    //Check if tracker has been calibrated
    while (calData[0] != 1){
      Serial.println("You have not calibrated your tracker");
      delay(1000);
      Serial.println("You MUST calibrate before the tracker will operate");
      delay(1000);
      Serial.println("Please see instructions to calibrate tracker");      
      delay(10000);
    }

}

// Check the tracker has been calibrated, if not go into loop.
// Its too dangerous for tracker or user to continue.
void checkCaled(){

    // Read Magnetometer calibration data
    int magCaled = eeprom_read(MAG_CAL_FLAG_ADDR);
    int accelCaled = eeprom_read(ACCEL_CAL_FLAG_ADDR);

    //Check if tracker has been calibrated
    while (magCaled != 1 || accelCaled != 1){
      Serial.println("You have not calibrated your tracker");
      delay(1000);
      Serial.println("You MUST calibrate before the tracker will operate");
      delay(1000);
      Serial.println("Please see instructions to calibrate tracker");      
      delay(10000);
    }

}
