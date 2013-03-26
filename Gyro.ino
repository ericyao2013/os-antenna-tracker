void ITG3200_init() {
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x3E);  
   Wire.write(0x00);   
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x15);  
   Wire.write(0x07);   
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x16);  
   Wire.write(0x1E);   // +/- 2000 dgrs/sec, 1KHz, 1E, 19
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x17);  
   Wire.write(0x00);   
   Wire.endTransmission(); 
   
   
}

void ITG3200_calibrate(){
  
 // Number of readings to take for average
 int number = 100

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 g_offx = 0;
 g_offy = 0;
 g_offz = 0;
 
 for (char i = 0;i<number;i++)
    {
    delay(10);  
    GyroRead();
    tmpx += GyroX;
    tmpy += GyroY;
    tmpz += GyroZ; 
    }  
 g_offx = tmpx/number;
 g_offy = tmpy/number;
 g_offz = tmpz/number;
 
}

void ITG3200_read() {
  Wire.beginTransmission(ITG3200_Address); 
  Wire.write(0x1B);
  Wire.endTransmission(); 
  
  Wire.beginTransmission(ITG3200_Address); 
  Wire.requestFrom(ITG3200_Address, 8);    // request 8 bytes from ITG3200
  
  int i = 0;
  byte buff[8];
  while(Wire.available())    
  { 
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission(); 
    
  GyroX = ((buff[4] << 8) | buff[5]) - g_offx;
  GyroY = ((buff[2] << 8) | buff[3]) - g_offy;
  GyroZ = ((buff[6] << 8) | buff[7]) - g_offz;
  GyroTemp = (buff[0] << 8) | buff[1]; // temperature 
  
   Serial.print("Gx=");
   Serial.print(GyroX / 14.375); // Data to Degree conversation
   Serial.print(", Gy=");
   Serial.print(GyroY / 14.375);
   Serial.print(", Gz=");
   Serial.print(GyroZ / 14.375);
   
   Serial.print(", Gt=");
   Serial.println(35+((GyroTemp+13200) / 280)) ;
  
}
