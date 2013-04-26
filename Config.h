/******************************************************************************/
/******************           Initial Setup           *************************/
/******************************************************************************/
//#define CALIBRATE_MAG
#define GYRO_CALIBRATE_STARTUP
//#define SET_LEVEL - Used for ACCEL, which I am not using at the moment.
//#define PRINT_TO_SERIAL //Should only be used for debugging.

/******************************************************************************/
/******************          Board Selection          *************************/
/******************************************************************************/
#define HK_MW_PRO
//#define OTHER_BOARDS //TODO

/******************************************************************************/
/******************         Servo Output pins         *************************/
/******************************************************************************/
//Analogue Pins only. TODO - Digital pins for unlimited rotation servos
/******************************************************************************/

#define PAN_PIN 10 //Pan Servo
#define TILT_PIN 9 //Tilt Servo
#define ROLL_PIN 8 //Roll Servo

/******************************************************************************/
/******************        Servo Output values        *************************/
/******************************************************************************/
//These are the minmum, middle and maximum values for each servo axis.
/******************************************************************************/

//PAN Servo should be able to move 360 Degrees or be externally geared to allow 360 degree movement.
//PAN_MIN is 180 degrees clockwise from the front of the tracker when facing its FRONT.
//PAN_MAX is 180 degrees couter clockwise from the front of the tracker when facing its FRONT.
#define PAN_MIN 1102
#define PAN_MAX 2010

//TILT servo should be able to move 90 Degrees.
//TILT_MIN should be at the bottom of travel and point parallel with the ground.
//TILT_MAX should be 180 degrees from TILT_MIN.
#define TILT_MIN 1040
#define TILT_MAX 1940

//ROLL servo should be able to move 10% more than the maximum ROLL required to counteract the vehicle the tracker is attached to, eg a Boat.
//Using the example of a Boat that will roll up to 40 degrees, servo required to move 88 degrees - 40 + 40 + (10% x (40 + 40)) = 88 degrees.
//If you have a servo that can move more than the required movement, it would be a good idea to use all the servo travel.
//ROLL_MIN (using example above) should be at 44 degrees to left from level.
//ROLL_MAX (using example above) should be 44 degrees to right from level.
#define ROLL_MIN 1037
#define ROLL_MAX 1943


/******************************************************************************/
/******************    Modem configuration options    *************************/
/******************************************************************************/
//This is the modem that is communicating with the vehicle

#define MODEM_SERIAL 3 //Serial Port modem is connected to
#define MODEM_BAUD 57600 //Baud rate modem is outputting at

/******************************************************************************/
/******************     GPS configuration options     *************************/
/******************************************************************************/
//Should stay uncommented as its required for tracker to work.
#define GPS

//GPS Protocol
#define NMEA //The only supported protocol at the moment
//#define UBLOX //TODO



//GPS Connection options - not options, just placeholders for TODO
//#define GPS_SERIAL
//#define GPS_I2C //TODO

//Serial Options
//#define GPS_SERIAL 2 - not really an option, just a placeholder for a TODO
#define GPS_BAUD 115200


/******************************************************************************/
/******************           Telemtry Types          *************************/
/******************************************************************************/
#define UAVTalk
//#define Mavlink //TODO

//Telemetry Versions
//UAVTalk
#define VERSION_RELEASE_12_10_2

//Mavlink
//TODO

/******************************************************************************/
/******************   GENERAL VARIABLES FOR SOFTWARE  *************************/
/******************************************************************************/
// Don't change these unless you know what you are doing
/******************************************************************************/

// MAG calibration values
static long MAG_MIN_X = 0;
static long MAG_MAX_X = 0;
static long MAG_MIN_Y = 0;
static long MAG_MAX_Y = 0;
static long MAG_MIN_Z = 0;
static long MAG_MAX_Z = 0;

// ACCEL calibration values
static long ACCEL_MIN_X = 0;
static long ACCEL_MAX_X = 0;
static long ACCEL_MIN_Y = 0;
static long ACCEL_MAX_Y = 0;
static long ACCEL_MIN_Z = 0;
static long ACCEL_MAX_Z = 0;

//Generic Variables
static long previousMillis = 0;
static long currentMillis = 0;


// TODO - Add code that monitors the battery on the HOME side and alarms when its too low.
static int      volt_div_ratio = 0;             // Volt * 100
static int      curr_amp_per_volt = 0;          // Ampere * 100
static int      curr_amp_offset = 0;            // Ampere * 10000

// Below variable could be used to ALARM on tracker side for low battery in no GCS environments.
static uint16_t osd_total_A = 0;                // Battery total current [mAh]

//Generic UAVTalk Variables
//static uint8_t  op_alarm = 0;                   // OP alarm info
//static uint8_t  veh_armed = 0;                  // OP armed info
//static bool     motor_armed = 0;

//GPS information from vehicle
static float        veh_lat = 0;                    // latidude
static float        veh_lon = 0;                    // longitude
static uint8_t      veh_satellites_visible = 0;     // number of satelites
static uint8_t      veh_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static uint8_t      veh_time_hour = 0;              // OP GPS time hour info
static uint8_t      veh_time_minute = 0;            // OP GPS tiem minute info

// Used for calculating Elevation.
static float        veh_alt = 0;                    // altitude

//Home information below will be set by the device running this code.
static bool         got_trac = 0;               // got home position or not
static int          trac_sats = 0;              // number of satellites
static float        trac_lat = 0;               // home latidude
static float        trac_lon = 0;               // home longitude
static float        trac_alt = 0;               // home altitude
static long         dist_to_veh = 0;            // distance from tracker to vehicle

//Tracker to Vehicle variables
//static uint8_t      azimuth_from_true_north = 0;    // azimuth from current tracker location
//static uint8_t      corrected_elevation = 0;        // elevation corrected for Home altitude and curvature of earth


// These variables could be used for the ROLL servo output. 
// Tracker could ROLL the antennas to keep polarity and/or counteract tracker location movements.
static int16_t      veh_pitch = 0;                  // pitch from DCM
static int16_t      veh_roll = 0;                   // roll from DCM
//The below variables not really needed.
static int16_t      veh_yaw = 0;                    // relative heading form DCM
static float        veh_heading = 0;                // ground course heading from GPS
static float        veh_groundspeed = 0;            // ground speed

//static float        veh_airspeed = -1;              // airspeed
//static float        veh_windspeed = 0;              // windspeed
//static float        veh_climb = 0;                  // climb rate

//These are channel numbers for the tracked vehicle side
//static uint16_t     veh_throttle = 0;               // throttle
//static uint16_t     ch_raw = 0;
//static int16_t      chan1_raw = 0;
//static int16_t      chan2_raw = 0;
//static int16_t      chan1_raw_middle = 0;
//static int16_t      chan2_raw_middle = 0;
//static uint16_t     veh_chan5_raw = 1000;
//static uint16_t     veh_chan6_raw = 1000;
//static uint16_t     veh_chan7_raw = 1000;
//static uint16_t     veh_chan8_raw = 1;
