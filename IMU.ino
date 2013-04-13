void getValues(float * values) {
  
  //accel array
  int axyz[3];
  
  // Get Accel Raw values
  accel.readAccel(axyz);
  values[0] = axyz[0];
  values[1] = axyz[1];
  values[2] = axyz[2];
  
  // remove offsets and scale accelerometer (calibration)
  values[0] = (values[0] - acc_off_x) / acc_scale_x;
  values[1] = (values[1] - acc_off_y) / acc_scale_y;
  values[2] = (values[2] - acc_off_z) / acc_scale_z;
  
  //Serial.print("SaccelX: ");Serial.print(acc_scale_x);Serial.print("  ");Serial.print("SaccelY: ");Serial.print(acc_scale_y);Serial.print("  ");Serial.print("SaccelZ: ");Serial.println(acc_scale_z);
  //Serial.print("accelX: ");Serial.print(values[0]);Serial.print("  ");Serial.print("accelY: ");Serial.print(values[1]);Serial.print("  ");Serial.print("accelZ: ");Serial.println(values[2]);
  
  // Gyro ********************************************
  
  //gyro array
  int gxyz[3];
  
  // Get Gyro Raw values
  gyro.readGyroRaw(gxyz);

  
  // Correct the Raw values for Offset, convert to Degrees/s then to radians
  values[3] = ((gxyz[0] - gyro_off_x) / 14.375) * DEG_TO_RAD;
  values[4] = ((gxyz[1] - gyro_off_y) / 14.375) * DEG_TO_RAD;
  values[5] = ((gxyz[2] - gyro_off_z) / 14.375) * DEG_TO_RAD;
  
  //Serial.print("gyroX: ");Serial.print(values[3]);Serial.print("  ");Serial.print("gyroY: ");Serial.print(values[4]);Serial.print("  ");Serial.print("gyroZ: ");Serial.println(values[5]);
  
  // Magnometer ********************************************
  
  //mag array
  int mxyz[3];
  
  // Get Mag Raw values
  mag.getRaw(&mxyz[0], &mxyz[1], &mxyz[2]);
    
  // Correct Mag Values for offset and scale
  values[6] = (mxyz[0] - magn_off_x) / magn_scale_x;
  values[7] = (mxyz[1] - magn_off_y) / magn_scale_y;
  values[8] = (mxyz[2] - magn_off_z) / magn_scale_z;

  //Serial.print("magX: ");Serial.print(values[6]);Serial.print("  ");Serial.print("magY: ");Serial.print(values[7]);Serial.print("  ");Serial.print("magZ: ");Serial.println(values[8]);
  //delay(500);

}

/*******************************************************************************************************/
void  AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  
  
  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;
    
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
  }


  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
  
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //Serial.print("q0: ");Serial.print(q0);Serial.print("  ");
  //Serial.print("q1: ");Serial.print(q1);Serial.print("  ");
  //Serial.print("q2: ");Serial.print(q2);Serial.print("  ");
  //Serial.print("q3: ");Serial.println(q3);
}

/*******************************************************************************************************/
void getQ(float * q) {
  
  // Array for sensor values
  float val[9];
  
  // Get the Sensor values
  getValues(val);
  
  now = micros();

  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  //Serial.print("SR: ");Serial.print(sampleFreq);Serial.print("Hz");  
  lastUpdate = now;
    
    // This is where you get the axis aligned.
    // gyro values are expressed in deg/sec
    #if defined(someboard)
      AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
    #elif defined(SEN_10724)
      AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[7], -val[6], val[8]);
    #elif defined(ARDUIMU_v3)
      AHRSupdate(val[3], val[4], val[5], val[0], val[1], val[2], -val[6], -val[7], val[8]);
    #elif defined(HK_MW_PRO)
    	//(      float gx, float gy, float gz,  float ax,  float ay, float az, float mx, float my,  float mz)
      AHRSupdate(-val[4],  val[3],   val[5],    -val[1],   val[0],   val[2],   val[6],   val[7],    val[8]);
    //Serial.print(val[3]);Serial.print(" ");Serial.print(val[3]);Serial.print(" ");Serial.print(val[5]);Serial.print(" ");
    //Serial.print(val[0]);Serial.print(" ");Serial.print(val[1]);Serial.print(" ");Serial.print(val[2]);Serial.print(" ");
    //Serial.print(-val[7]);Serial.print(" ");Serial.print(val[6]);Serial.print(" ");Serial.println(val[8]);      
    #endif

  
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
  
}

/*******************************************************************************************************/
void getYawPitchRollRad(float * ypr) {
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  getQ(q);
  
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}

void getYawPitchRoll(float * ypr) {
  getYawPitchRollRad(ypr);
  arr3_rad_to_deg(ypr);
}

void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/M_PI;
  arr[1] *= 180/M_PI;
  arr[2] *= 180/M_PI;
}

/*******************************************************************************************************/
float invSqrt(float number) {
  union {
    float f;
    int32_t i;
  } y;

  y.f = number;
  y.i = 0x5f375a86 - (y.i >> 1);
  y.f = y.f * ( 1.5f - ( number * 0.5f * y.f * y.f ) );
  return y.f;
}
