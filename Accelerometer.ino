#if defined(ACCEL)

#if defined(BMA190)

void BMA180_init(){ 
 byte temp[1];
 byte temp1;
  //
  i2c_write(BMA180_ADDRESS,BMA180_RESET,0xB6);
  
  //wake up mode
  i2c_write(BMA180_ADDRESS,BMA180_PWR,0x10);
  
  // low pass filter,
  i2c_read(BMA180_ADDRESS, BMA180_BW, 1 ,temp);
  temp1=temp[0]&0x0F;
  i2c_write(BMA180_ADDRESS BMA180_BW, temp1);   
  
  // range +/- 2g 
  i2c_read(BMA180_ADDRESS, BMA180_RANGE, 1 , temp);  
  temp1=(temp[0]&0xF1) | 0x04;
  i2c_write(BMA180_ADDRESS, BMA180_RANGE, temp1);
}

void ACCEL_calibrate(){
  
 // Number of readings to take for average
 int number = 100

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 a_offx = 0;
 a_offy = 0;
 a_offz = 0;
 
 for (int i = 0; i < number; i++)
    {
    delay(10);  
    ACCEL_read();
    tmpx += AccelX;
    tmpy += AccelY;
    tmpz += AccelZ; 
    }  
 a_offx = tmpx/number;
 a_offy = tmpy/number;
 a_offz = tmpz/number;
 
}
  


void ACCEL_read(){ 
 // read in the 3 axis data, each one is 14 bits 
 uint8_t scaled[3];
 
 unit8_t* buffer = i2c_read(BMA180_ADDRESS, BMA180_DATA, 6);
 
   scaled[0] = (( buffer[0] | buffer[1]<<8)>>2) + offx ;

   scaled[1] = (( buffer[2] | buffer[3]<<8 )>>2) + offy;

   scaled[2] = (( buffer[4] | buffer[5]<<8 )>>2) + offz;

 return scaled;
}

#endif //BMA180

#endif //ACCEL
