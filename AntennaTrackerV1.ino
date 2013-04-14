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


  // Check tracker has been calibrated
  // Tracker will stop at this point if not calibrated
  //checkCaled();

}


float DistanceBetween(float lat1, float lon1, float lat2, float lon2){

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
// Error of approx 1 degrees is the best this formula and AVR chip can get.
// Taking into account earth gravity effect on radio waves would reduce the error by about 1 degree for every 200km.
// In saying the above, the AVR chip produces quite a bit of error (only 7 decimal places).
float ElevationAngle(float distance, float alt2, float alt2){

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
float Azimuth(float distance, float lat1, float lon1, float lat2, float lon2){

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

void MoveServos(float eAngle, float azimuth, float tilt, float roll, float tnHeading){

  // Because most Pan servos can only move 360 degrees max, the midpoint of the PAN servo is the front of the tracker.
  // If the tracker is pointing +90 degrees (East), all measurements will be offset by -90 degrees to keep the maximum
  // amount of travel available.

  // Correct azimuth for True North Heading (Heading compensated for declination)
  float correctedAzimuth = azimuth - tnHeading;

  // Correct Elevation angle for Tilt
  float correctedTilt = eAngle - tilt;

  // Mapping Servo Min and Max to the MIN and MAX of the Axis
  int servoPan = map(correctedAzimuth, 0, 359, PAN_MIN, PAN_MAX);
  
  int servoTilt = map(eAngle, 0, 90, TILT_MIN, TILT_MAX);

  int servoRoll = map(azimuth, 0, 359, ROLL_MIN, ROLL_MAX);

  // Moving the Servos
  PAN.writeMicroseconds(servoPan);  
  TILT.writeMicroseconds(servoTilt);
  ROLL.writeMicroseconds(servoRoll);

}

//The main loop, where everything happens.
//It is important that this loop be as quick as possible so we don't loose UAVTalk packets.
void loop(){
  
  // Get GPS Data
  getGPS();
  
  // Only do the below if we have local GPS data and Vehicle GPS data
  if (gpsStat == 1 && telStat == 1){
    // Used for Elevation Angle
    float distanceToHome = DistanceBetween();
  
    // Get the Elevation angle
    float ElevAngle = ElevationAngle(distanceToHome);
  
    // Get the Azimuth angle
    float AzAngle = Azimuth(distanceToHome, trac_lat, trac_lon, veh_lat, veh_lon);
    
    // Get the Declination Angle
    float declinationAngle = declination.get_declination(trac_lat, trac_lon);
    
  }

  // AHRS Array
  float ypr[3];
  // Get AHRS data - Yaw, Pitch, Roll
  getYawPitchRoll(ypr);
  
  moveServos();

#if defined (PRINT_TO_SERIAL)
  serialPrint();
#endif

}

void moveServos(int imuStat, int gpsStat, int telStat, float yaw, float pitch, float roll){ 
  
  // If IMU, GPS and Telemetry Status = Ready then move servos
  if (imuStat == 1 && gpsStat == 1 && telStat == 1){
  //Calulate Servo movements
  int panMicroseconds = map(yaw, -179, 179, PanMinPWM, PanMaxPWM);
  int tiltMicroseconds = map(pitch, 89, -89, TiltMinPWM, TiltMaxPWM);
  int rollMicroseconds = map(roll, 89, -89, RollMinPWM, RollMaxPWM);
  
  // Move Servos
  PAN.writeMicroseconds(panMicroseconds);
  TILT.writeMicroseconds(tiltMicroseconds);
  ROLL.writeMicroseconds(rollMicroseconds);
  }
  else{
    // Move the servos to default starting position
    servoDefault();
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


