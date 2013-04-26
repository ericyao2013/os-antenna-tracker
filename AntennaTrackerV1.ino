/******************************************************************************/
/******************           Included Files          *************************/
/******************************************************************************/
//Standard Arduino Library
#include "Arduino.h"
//EEPROM Library
#include "EEPROM.h"
//Configuration options and variable declarations - This is where you setup your configuration
#include "Config.h"
//Definitions file - no user configurable options.
#include "Defs.h"
//Variables file - no user configurable options.
#include "Vars.h"
//UAVTalk Version information file
#include "UAVTalk.h"
//Arduino Library for complex Math functions
#include "math.h"
//Arduino I2C library
#include "Wire.h"
// Arduino Servo library
#include "Servo.h"
// Declination library
#include "AP_Declination.h"
// TinyGPS Library
#include <TinyGPS.h>
// HMC5883L Library
#include "HMC58X3.h"
// BMA180 Library
#include "bma180.h"
// IT3200 Library - ITG3205 uses the same library
#include "ITG3200.h"
/******************************************************************************/

// Declination object
AP_Declination declination;
// TinyGPS object
TinyGPS gps;
//void getgps(TinyGPS &gps); // No idea what this does - seems to be an array return
// HMC5883L object
HMC58X3 mag = HMC58X3();
// BMA180 object
BMA180 accel = BMA180();
// ITG3200 object
ITG3200 gyro = ITG3200();

Servo PAN;
Servo TILT;
Servo ROLL;

int calData[16];

void setup(){

  //Get the start time - used for servo output initialisation.
  //Won't be required when there is the ability to ARM the tracker (most likely via button).
  currentMillis = millis();

  // Setup the Serial Connections.
  Serial.begin(115200);
  Serial2.begin(MODEM_BAUD);
  Serial3.begin(GPS_BAUD);
  
  //Initialising the i2c interface
  Wire.begin();

  // Attach/Setup Servos
  PAN.attach(PAN_PIN);
  TILT.attach(TILT_PIN);
  TILT.attach(TILT_PIN);

  // Initialise the ACCEL
  ACCEL_init();
  //Initialise the MAG
  MAG_init();
  //Initialise the GYRO
  GYRO_init();

  #if defined(GYRO_CALIBRATE_STARTUP)
    GYRO_calibrate();
  #endif
  
  #if defined(ACCEL_CALIBRATE)
    //ACCEL_calibrate(); // 6 Point calibration
    ACCEL_calibrateQ(); // Quick level calibration
  #endif
  
  #if defined(MAG_CALIBRATE)
    MAG_calibrate();
  #endif

  // Start Timer for IMU Status
  startTimer = millis();

}


float DistanceBetween (float lat1, float lon1, float lat2, float lon2){

  float ToRad = PI / 180.0;

  // Radius of the Earth in meters
  // We do not account for Altitude until elevation angle is calculated.
  float Radius = 6371000;

  float dLat = (lat2-lat1) * ToRad;
  float dLon = (lon2-lon1) * ToRad;

  float a = sin(dLat/2) * sin(dLat/2) +
    cos(lat1 * ToRad) * cos(lat2 * ToRad) *
    sin(dLon/2) * sin(dLon/2);

  float c = 2 * atan2(sqrt(a), sqrt(1-a));

  float distance = Radius * c;
  return distance;

}


// Here we get the Elevation angle
// To take into account altitude of each vehicle the formula uses great circle
// but uses the difference in altitudes to get proper estimation of elevation.
float ElevationAngle (float distance, float alt1, float alt2){

  // Convert Radians to Degrees
  float ToDeg = 180.0 / PI;

  // Radius of the Earth in meters
  float Radius = 6371000;

  float eAngle = ToDeg * ( (alt2 - alt1) / distance - (distance / (2*Radius)) );

  // Returning the Elevation Angle 
  return eAngle;

}

// Getting the Azimuth Angle
// Error of approx 1 degrees is the best this formula and AVR chip can get
float Azimuth (float distance, float lat1, float lon1, float lat2, float lon2){

  float AzAngle = 0;

  float x = acos( (sin(lat2) - sin(lat1)*cos(distance) ) / (sin(distance)*cos(lat2)) );

  if (sin(lon2-lon1) < 0) {
    AzAngle = x;
  }
  else if (sin(lon2-lon1) > 0) {
    AzAngle = 2 * PI - x;
  }

  return AzAngle;

}

//The main loop, where everything happens.
void loop(){
  
  float correctedAzimuth = 0.0;
  float correctedTilt = 0.0;
  float correctedRoll = 0.0;
  
  // Get GPS Data
  getGPS();
  
  // AHRS Array
  float ypr[3];
  // Get AHRS data - Yaw, Pitch, Roll
  getYawPitchRoll(ypr);
  
  // IMU needs time to settle - give it 45 seconds
  if((startTimer + 45000) < millis()){
  imuStat = 1;
  }
  
  
  // Only do the below if we have local GPS data and Vehicle GPS data
  // The reason is we need the data for local and vehicle GPS data to get a result.
  if (gpsStat == 1 && telStat == 1){
    // Used for Elevation Angle
    float distanceToHome = DistanceBetween(trac_lat, trac_lon, veh_lat, veh_lon);
  
    // Get the Elevation angle
    float ElevAngle = ElevationAngle(distanceToHome, trac_alt, veh_alt);
  
    // Get the Azimuth angle
    float AzAngle = Azimuth(distanceToHome, trac_lat, trac_lon, veh_lat, veh_lon);
    
    // Get the Declination Angle
    float declinationAngle = declination.get_declination(trac_lat, trac_lon);
  
  // Because most Pan servos can only move 360 degrees max, the midpoint of the PAN servo is the front of the tracker.
  // For example - if the tracker is pointing +90 degrees (East), all measurements will be offset by -90 degrees to
  // keep the maximum amount of travel available.

  // Correct Azimuth to Vehicle for declination and Yaw.
  correctedAzimuth = AzAngle - ypr[0] + declinationAngle;

  // Correct Elevation angle for Tilt/Roll
  // We correct Tilt with Roll because if the tracker is rolled by 10 degrees and vehicle is due East, the Elevation Angle needs to be 
  // corrected for Roll instead of Pitch. We correct it by getting the corrected Azimuth and using it calculate the 
  // percentage of roll/tilt to correct Elevation.
  
  // TILT COMPENSATION
  // If correctedAzimuth is between -90 and -180 or +90 and +180
  if (correctedAzimuth > 90){
    // correctedTilt = Elevation Angle + Percentage of Roll to Apply + Percentage of Pitch to Apply
    correctedTilt = ElevAngle + (ypr[2] * (180 - correctedAzimuth)) + (ypr[1] * (abs(ypr[1]) - 180));
  }
  // If yaw is between -90 and +90 degrees
  else{
    // correctedTilt = Elevation Angle + Percentage of Roll to Apply + Percentage of Tilt to Apply
    correctedTilt = ElevAngle + (ypr[2] * correctedAzimuth) + (correctedAzimuth * ypr[2]);
  }
  
  // ROLL COMPENSATION
  // If correctedAzimuth is between -90 and -180 or +90 and +180
  if (abs(ypr[0]) > 90){
    // correctedTilt = Elevation Angle + Percentage of Roll to Apply + Percentage of Pitch to Apply
    correctedRoll = ElevAngle + (ypr[2] * (180 - correctedAzimuth)) + (ypr[1] * (abs(ypr[1]) - 180));
  }
  // If correctedAzimuth is between -90 and +90 degrees
  else{
    // correctedTilt = Elevation Angle + Percentage of Roll to Apply + Percentage of Tilt to Apply
    correctedRoll = ElevAngle + (ypr[2] * correctedAzimuth) + (correctedAzimuth * ypr[2]);
  }
  
  // Move Servos
  moveServos(correctedAzimuth, correctedTilt, correctedRoll);  
  
  } // End Only do when have all data.

#if defined (PRINT_TO_SERIAL)
  serialPrint();
#endif

}

void moveServos(float yaw, float pitch, float roll){ 
  
  // If IMU, GPS and Telemetry Status = Ready then move servos
  if (imuStat == 1 && gpsStat == 1 && telStat == 1){
  //Calulate Servo movements
  int panMicroseconds = map(yaw, -179, 179, PAN_MIN, PAN_MAX);
  int tiltMicroseconds = map(pitch, 89, -89, TILT_MIN, TILT_MAX);
  int rollMicroseconds = map(roll, 89, -89, ROLL_MIN, ROLL_MAX);
  
  // Move Servos
  PAN.writeMicroseconds(panMicroseconds);
  TILT.writeMicroseconds(tiltMicroseconds);
  ROLL.writeMicroseconds(rollMicroseconds);
  }
  else{
      // Move the servos to default starting position
      PAN.writeMicroseconds(PAN_MIN + ((PAN_MAX - PAN_MIN) / 2));
      TILT.writeMicroseconds(TILT_MIN + ((TILT_MAX - TILT_MIN) / 2));
      ROLL.writeMicroseconds(ROLL_MIN + ((ROLL_MAX - ROLL_MIN) / 2));
  }
}

// Flash LEDs visually shows the status of the tracker.
// RestLED - Status of whole system - Solid when not ready and flash once per second when ready.
// LED1 - IMU Software - flash once per second when not ready and solid when ready.
// LED2 - Local GPS Status - flash once per second when not ready and solid when ready.
// LED3 - Telemetry Status - flash once per second when not ready and solid when ready.
void flashLeds(){
  
  if (imuStat == 0 || gpsStat == 0 || telStat == 0) {
    digitalWrite(SYSLEDPIN, HIGH);
  }
  else{
    digitalWrite(SYSLEDPIN, LEDState);
  }
  
  // If IMU is Ready then solid LED else flash LED
  if (imuStat == 1) {
    digitalWrite(IMULEDPIN, LOW);
  }
  else {
    digitalWrite(IMULEDPIN, LEDState);
  }
  
  // If GPS is Ready then solid LED else flash LED
  if (gpsStat == 1) {
    digitalWrite(GPSLEDPIN, LOW);
  }
  else {
    digitalWrite(GPSLEDPIN, LEDState);
  }

  // If Telemetry is Ready then solid LED else flash LED
  if (telStat == 1) {
    digitalWrite(TLEDPIN, LOW);
  }
  else {
    digitalWrite(TLEDPIN, LEDState);
  }
  
  // Change value of LED Pin
  if (LEDState == 0){
    LEDState = 1;
  }
  else{
      LEDState = 0;
    }
}


