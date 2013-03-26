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
#include "UAVTalkVersions.h"
//Standard Arduino Library for complex Math functions
#include "math.h"
//Arduino I2C library. Not completely necessary and could be removed if code was updated to bypass library.
#include "Wire.h"
// Arduino Servo library
#include "Servo.h"
/******************************************************************************/


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
/*
void GetEepromValues(){
  //Only need EEPROM Address information
  //Put delay to make sure read is successful.
  
  #if defined(MAG)
  //Get MAG data
  MAG_MIN_X = EepromRead(MAG_MIN_X_ADDR);
  delay(5);
  MAG_MAX_X = EepromRead(MAG_MAX_X_ADDR);
  delay(5);
  MAG_MIN_Y = EepromRead(MAG_MIN_Y_ADDR);
  delay(5);
  MAG_MAX_Y = EepromRead(MAG_MAX_Y_ADDR);
  delay(5);
  MAG_MIN_Z = EepromRead(MAG_MIN_Z_ADDR);
  delay(5);
  MAG_MAX_Z = EepromRead(MAG_MAX_Z_ADDR);
  delay(5);
  #endif
  
  #if defined(ACCEL)
  ACCEL_OFFSET_X = EepromRead(ACCEL_OFFSET_X_ADDR);
  delay(5);
  ACCEL_OFFSET_Y = EepromRead(ACCEL_OFFSET_Y_ADDR);
  delay(5);  
  ACCEL_OFFSET_Z = EepromRead(ACCEL_OFFSET_Z_ADDR);
  #endif
  
}

void CheckCalibrated(){
  
  //Check to make sure MAG Configuration data NOT NULL
  
  //Check to make sure ACCEL Configuration data NOT NULL
  if (!ACCEL_OFFSET_X || !ACCEL_OFFSET_X || !ACCEL_OFFSET_X) { //ACCEL always has offset.
    //Do something.
  }
  
}
*/
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


// Elevation angle is not going to be perfect but it will be close enough.
// Error of approx 1-2 degrees is 
float ElevationAngle(float distance){

  // Convert Radians to Degrees
  float ToDeg = 180.0 / PI;
  
  // Radius of the Earth in meters
  float Radius = 6371000;
  
  //Forumula found here - http://tchester.org/sgm/analysis/peaks/how_to_get_view_params.html
  
  float lambda = ToDeg * ( (veh_alt - trac_alt) / distance - (distance / (2*Radius)) );
  
  return lambda;
  
}

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

void MoveServos(float eangle, float azimuth){
  
  float tilt = map(eangle, 0, 90, TILT_MIN, TILT_MAX);
  
  float pan = map(azimuth, 0, 359, PAN_MIN, PAN_MAX);
  
  
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
    
    //GET MAG HEADING BEFORE SENDING TO SERVOS.
    //MAG heading must be corrected for:
    //Declination
    int magheading = 1;
    
    //Servos must be corrected for current front as Zero point.
    MoveServos(ElevAngle, AzAngle);
  
    //Only run the following code if its been 1 second since the last time it was run.
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > 1000) {
      previousMillis = currentMillis;
    
    #if defined (PRINT_TO_SERIAL)
    serialPrint();
    #endif
    }
}
