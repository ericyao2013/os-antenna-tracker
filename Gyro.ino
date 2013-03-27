void GYRO_init() {

  gyro.init(ITG3200_ADDR_AD0_LOW);
   
}

void GYRO_calibrate(){
  
  //You should not move the board during the below
  gyro.zeroCalibrate(2500,2);
 
}

void GYRO_read() {
  
  // Read GYRO temperature
  gyro.readTemp(&temperature);
  // Read GYRO Values
  gyro.readGyro(&x,&y,&z);
  
}

#endif //ITG3200

#endif //GYRO
