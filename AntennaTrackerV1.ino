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
#include "TinyGPS.h"
// HMC5883L Library
#include "HMC5883L.h"
// BMA180 Library
#include "bma180.h"
// IT3200 Library - ITG3205 uses the same library
#include "ITG3200.h"
/******************************************************************************/

// Declination object
AP_Declination declination;
// TinyGPS object
TinyGPS gps;
void getgps(TinyGPS &gps); // No idea what this does - seems to be an array return
// HMC5883L object
HMC5883L mag = HMC5883L();
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

  PAN.attach(PAN_PIN);
  TILT.attach(TILT_PIN);

  // Initialise the ACCEL
  ACCEL_init();
  //Initialise the MAG
  MAG_init();
  //Initialise the GYRO
  GYRO_init();

#if defined(GYRO_CALIBRATE_STARTUP)
  GYRO_setLevel();
#endif

#if defined(ACCEL_CALIBRATE)
  ACCEL_calibrate();
#endif

#if defined(MAG_CALIBRATE)
  MAG_calibrate();
#endif


  // Check tracker has been calibrated
  // Tracker will stop at this point if not calibrated
  checkCaled();

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
float ElevationAngle(float distance){

  // Convert Radians to Degrees
  float ToDeg = 180.0 / PI;

  // Radius of the Earth in meters
  float Radius = 6371000;

  float eAngle = ToDeg * ( (veh_alt - trac_alt) / distance - (distance / (2*Radius)) );

  // Returning the Elevation Angle 
  return eAngle;

}

// Getting the Azimuth Angle
// Error of approx 1 degrees is the best this formula and AVR chip can get
float Azimuth(float distance, float lat1, float lon1, float lat2, float lon2){

  float AzAngle = 0;

  float x = acos( (sin(lat2) - sin(lat1)*cos(distance) ) / (sin(distance)*cos(lat1)) );

  if (sin(lon2-lon1) < 0) {
    AzAngle = x;
  }
  else if (sin(lon2-lon1) > 0) {
    AzAngle = 2 * PI - x;
  }

  return AzAngle;

}


// Here we calculate the Tilt and Roll of the antenna tracker - using Kalman filter in IMU Tab for this
byte* TiltRoll(byte *data){

  // First 3 values are ACCEL, last 2 are GYRO values
  double zeroValue[5] = { 0, 0, 0, -32, 254  }; // TODO - Get these values during ACCEL and GYRO calibration.

  /* All the angles start at 180 degrees */
  double gyroXangle = 180;
  double gyroYangle = 180;

  double compAngleX = 180.0;
  double compAngleY = 180.0;

  // Timer for Gyro readings
  unsigned long timer;
  timer = micros();


  // Create GYRO array
  int xyz[3];
  // Read GYRO Values
  gyro.readGyroRaw(xyz);

  int gyroRawX = (xyz[0]);
  int gyroRawY = (xyz[1]);
  int gyroRawZ = (xyz[2]);

  // Convert Gyro Raw (X) into Gyro Degrees per second
  double gyroXrate = ((gyroRawX-zeroValue[3])/14.375);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000);

  // Convert Gyro Raw (Y) into Gyro Degrees per second
  double gyroYrate = ((gyroRawY-zeroValue[4])/14.375);
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);

  double accXangle = getXangle();
  double accYangle = getYangle();

  double xAngle = kalmanX(accXangle, gyroXrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter
  double yAngle = kalmanY(accYangle, gyroYrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter

  timer = micros();

  // Put the TILT/ROLL angles into the array
  data[0] = xAngle;
  data[1] = yAngle;

  Serial.print("Xangle: ");Serial.print(xAngle);Serial.print("  Yangle: ");Serial.println(yAngle);
  
  return data;

}


void MoveServos(float eAngle, float azimuth, float tilt, float roll, float tnHeading){

  // Because most Pan servos can only move 360 degrees max, the midpoint of the servos is the front of the tracker.
  // If the tracker is pointing East 90 degrees, all measurements will be offset by 90 degrees to keep the maximum
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
  
  Serial.print("Pan PWM: ");Serial.println(servoPan);
  Serial.print("Tilt PWM: ");Serial.println(servoTilt);
  Serial.print("Roll PWM: ");Serial.println(servoRoll);

}

//The main loop, where everything happens.
//It is important that this loop be as quick as possible so we don't loose UAVTalk packets.
void loop(){

  // Used for Elevation Angle
  float distanceToHome = DistanceBetween(trac_lat, trac_lon, veh_lat, veh_lon);

  // Used for Servo Movement
  float ElevAngle = ElevationAngle(distanceToHome);

  // Used for Servo Movement
  float AzAngle = Azimuth(distanceToHome, trac_lat, trac_lon, veh_lat, veh_lon);
  
  // Get the Declination Angle
  float declinationAngle = declination.get_declination(trac_lat, trac_lon);  

  // AHRS Array
  float ypr[3];
  // Get AHRS data
  //my3IMU.getYawPitchRoll(ypr);
  
  // Put Angles array into variables to pass to MoveServos()
  float pitchDegrees = ypr[1];
  float rollDegrees = ypr[2];
  float yawDegrees = ypr[0] + declinationAngle;  

  //Servos must be corrected for current front as Zero point.
  MoveServos(ElevAngle, AzAngle, pitchDegrees, rollDegrees, yawDegrees);

#if defined (PRINT_TO_SERIAL)
  serialPrint();
#endif

}

