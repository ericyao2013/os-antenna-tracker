void ACCEL_init(){
  
  bma180.SetAddress((int)BMA180_ADDRESS_SDO_LOW);
  bma180.SoftReset();
  bma180.enableWrite();
  bma180.SetFilter(bma180.F10HZ);
  bma180.setGSensitivty(bma180.G2);
  bma180.SetSMPSkip();
  bma180.SetISRMode();
  bma180.disableWrite();
  delay(100);
  
}


