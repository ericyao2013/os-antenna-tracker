/******************************************************************************/
/******************           Included Files          *************************/
/******************************************************************************/
//Standard Arduino Library
#include "Arduino.h"
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
/******************************************************************************/

// Declination object
AP_Declination declination;
// TinyGPS object
TinyGPS gps;


Servo PAN;
Servo TILT;
Servo ROLL;

void setup(){
  
  //Get the start time - used for servo output initialisation.
  //Won't be required when there is the ability to ARM the tracker (most likely via button).
  currentMillis = millis();
  
  //Setup the Serial Connections.
  Serial.begin(9600);
  Serial2.begin(MODEM_BAUD);
  Serial3.begin(GPS_BAUD);
  
  PAN.attach(PAN_PIN);
  TILT.attach(TILT_PIN);
  
  #if defined(CALIBRATE_MAG)
  MAG_calibrate();
  #endif
  
  // Read the MAG calibration values
  // Values are set when the MAG is calibrated
  readMagCal();
  //Get stored data from EEPROM
  //GetEepromValues();
  
  //Check to make sure calibration has been done.
  //Tracker will not operate without calibration being done.
  //CheckCalibrated();

}


void serialPrint(){
  
          Serial.print("UAVTalk State: ");Serial.println(uavtalk_state());
          Serial.print("Fix Type: ");Serial.println(veh_fix_type);
          Serial.print("Latitude: ");Serial.println(veh_lat,7);
          Serial.print("Longitude: ");Serial.println(veh_lon,7);
          Serial.print("Altitude: ");Serial.println(veh_alt,7);
          Serial.print("# Sats: ");Serial.println(veh_satellites_visible);
          
          Serial.print("Trac Lat: ");Serial.println(trac_lat);
          Serial.print("Trac Lon: ");Serial.println(trac_lon);
          Serial.print("Trac Alt: ");Serial.println(trac_alt);          
          Serial.println();
          Serial.println();
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
// Error of approx 1 degrees is the best this formula and AVR chip can get
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


// Here we get the MAG readings and the ACCEL readings and then tilt compensate them.
// We then get the Declination and compensate the heading for that
// Returns the Heading from true north
float Heading(){
  
  // Get MAG scaled values
  byte* magValues = MAG_read();
  
  // Get ACCEL scaled Values
  byte* accelValues = ACCEL_read();
  
  // Get Tilt compensated heading
  float headingTiltComp = CalculateHeadingTiltComp(magValues, accelValues);
  
  // Get the Declination Angle
  float declinationAngle = declination.get_declination(trac_lat, trac_lon);
  
  // Correct heading by Declination Angle
  float tnHeading = headingTiltComp + declinationAngle;
  
  return tnHeading;

}

  
//Get tilt compensated heading
// Code found here - 
float CalculateHeadingTiltComp(magValues, accelValues){
    
  // We are swapping the accelerometers axis as they are opposite to the compass the way we have them mounted.
  // We are swapping the signs axis as they are opposite.
  // Configure this for your setup.
  float accX = -acc.YAxis;
  float accY = -acc.XAxis;
  
  float rollRadians = asin(accY);
  float pitchRadians = asin(accX);
  
  // We cannot correct for tilt over 40 degrees with this algorithm, if the board is tilted as such, return 0.
  if(rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
  {
    return 0;
  }
  
  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(rollRadians);
  float sinRoll = sin(rollRadians);  
  float cosPitch = cos(pitchRadians);
  float sinPitch = sin(pitchRadians);
  
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
  
  float heading = atan2(Yh, Xh);
    
  return heading;
  
  return tcheading;
  
}  


// Here we calculate the Tilt and Roll of the antenna tracker
float TiltRoll(){
  
  // Get MAG scaled values
  float magValues = MAG_read();
  
  // Get ACCEL scaled Values
  float accelValues = ACCEL_read();
 
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
    int servoTilt = map(eangle, 0, 90, TILT_MIN, TILT_MAX);
    
    int servoPan = map(correctedAzimuth, 0, 359, PAN_MIN, PAN_MAX);
    
    int ServoRoll = map(azimuth, 0, 359, ROLL_MIN, ROLL_MAX);
 
    // Moving the Servos
    TILT.writeMicroseconds(servoTilt);
    PAN.writeMicroseconds(servoPan);  
    ROLL.writeMicroseconds(servoRoll);  
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
    
    // Get corrected compass heading
    float tnHeading = Heading()
    
    // Get Tilt & Roll
    //float tiltDegrees = ACCEL_read()
    
    //Servos must be corrected for current front as Zero point.
    MoveServos(ElevAngle, AzAngle, tiltDegrees, rollDegrees, tnHeading);
    
    #if defined (PRINT_TO_SERIAL)
    serialPrint();
    #endif
    }
}
