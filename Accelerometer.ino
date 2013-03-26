void BMA180_init(){ 
 byte temp[1];
 byte temp1;
  //
  i2c_Write(BMA180_ADDRESS,BMA180_RESET,0xB6);
  
  //wake up mode
  i2c_Write(BMA180_ADDRESS,BMA180_PWR,0x10);
  
  // low pass filter,
  i2c_Read(BMA180_ADDRESS, BMA180_BW,1,temp);
  temp1=temp[0]&0x0F;
  i2c_Write(BMA180_ADDRESS BMA180_BW, temp1);   
  
  // range +/- 2g 
  i2c_Read(BMA180_ADDRESS, BMA180_RANGE, 1 ,temp);  
  temp1=(temp[0]&0xF1) | 0x04;
  writeTo(BMA180_ADDRESS, BMA180_RANGE, temp1);
}

void BMA180_calibrate(){
  
void ITG3200_calibrate(){
  
 // Number of readings to take for average
 int number = 100

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 a_offx = 0;
 a_offy = 0;
 a_offz = 0;
 
 for (char i = 0;i<number;i++)
    {
    delay(10);  
    BMA180_read();
    tmpx += GyroX;
    tmpy += GyroY;
    tmpz += GyroZ; 
    }  
 g_offx = tmpx/number;
 g_offy = tmpy/number;
 g_offz = tmpz/number;
 
}
  
}

void BMA180_read(){ 
 // read in the 3 axis data, each one is 14 bits 
 // print the data to terminal 
 int n=6;
 byte result[5];
 i2c_Read(BMA180_ADDRESS, BMA180_DATA, n , result);
 
 int x= (( result[0] | result[1]<<8)>>2)+offx ;
 float x1=x/0;//4096.0;
  Serial.print("Ax=");
 Serial.print(x);
 Serial.print("g"); 
 //
 int y= (( result[2] | result[3]<<8 )>>2)+offy;
 float y1=y/0;//4096.0;
 Serial.print(", Ay=");
 Serial.print(y);
 Serial.print("g"); 
 //
 int z= (( result[4] | result[5]<<8 )>>2)+offz;
 float z1=z/0;//4096.0;
 Serial.print(", Az=");
 Serial.print(z);
 Serial.println("g"); 
}
