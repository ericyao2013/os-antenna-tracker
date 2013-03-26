#if defined(GPS)

#if defined(INIT_MTK_GPS)
  //#define MTK_SET_BINARY          PSTR("$PGCMD,16,0,0,0,0,0*6A\r\n")
  #define MTK_SET_NMEA            PSTR("$PGCMD,16,1,1,1,1,1*6B\r\n")
  //#define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
  //#define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
  //#define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
  #define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")
  #define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n") // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s  
  #define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
  #define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
  #define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")  //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)
#endif

#if defined(GPS_SERIAL) 
 #if defined(INIT_MTK_GPS) || defined(UBLOX)
  uint32_t init_speed[5] = {9600,19200,38400,57600,115200};
  void SerialGpsPrint(prog_char* str) {
    char b;
    while(str && (b = pgm_read_byte(str++))) {
      SerialWrite(GPS_SERIAL, b); 
      #if defined(UBLOX)
        delay(5);
      #endif      
    }
  }
 #endif

  void GPS_SerialInit() {
    SerialOpen(GPS_SERIAL,GPS_BAUD);
    delay(1000);
    #if defined(UBLOX)
      for(uint8_t i=0;i<5;i++){
        SerialOpen(GPS_SERIAL,init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
        #if (GPS_BAUD==19200)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
        #endif  
        #if (GPS_BAUD==38400)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
        #endif  
        #if (GPS_BAUD==57600)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
        #endif  
        #if (GPS_BAUD==115200)
          SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
        #endif  
        while(!SerialTXfree(GPS_SERIAL)) delay(10);
      }
      delay(200);
      SerialOpen(GPS_SERIAL,GPS_BAUD);  
      for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
        SerialWrite(GPS_SERIAL, pgm_read_byte(UBLOX_INIT+i));
        delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
      }
    #elif defined(INIT_MTK_GPS)                              // MTK GPS setup
      for(uint8_t i=0;i<5;i++){
        SerialOpen(GPS_SERIAL,init_speed[i]);                // switch UART speed for sending SET BAUDRATE command
        #if (GPS_BAUD==19200)
          SerialGpsPrint(PSTR("$PMTK251,19200*22\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
        #endif  
        #if (GPS_BAUD==38400)
          SerialGpsPrint(PSTR("$PMTK251,38400*27\r\n"));     // 38400 baud
        #endif  
        #if (GPS_BAUD==57600)
          SerialGpsPrint(PSTR("$PMTK251,57600*2C\r\n"));     // 57600 baud
        #endif  
        #if (GPS_BAUD==115200)
          SerialGpsPrint(PSTR("$PMTK251,115200*1F\r\n"));    // 115200 baud
        #endif  
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      }
      // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
      // So now we have to set the desired mode and update rate (which depends on the NMEA or MTK_BINARYxx settings)
      SerialOpen(GPS_SERIAL,GPS_BAUD);

      SerialGpsPrint(MTK_NAVTHRES_OFF);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(SBAS_ON);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(WAAS_ON);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(SBAS_TEST_MODE);
        while(!SerialTXfree(GPS_SERIAL)) delay(80);
      SerialGpsPrint(MTK_OUTPUT_5HZ);           // 5 Hz update rate

      #if defined(NMEA)
        SerialGpsPrint(MTK_SET_NMEA_SENTENCES); // only GGA and RMC sentence
      #endif     
      #if defined(MTK_BINARY19) || defined(MTK_BINARY16)
        SerialGpsPrint(MTK_SET_BINARY);
      #endif


    #endif  //elif init_mtk_gps
  }
#endif //gps_serial

void GPS_NewData() {
  uint8_t axis;
  #if defined(GPS_SERIAL) 
    #if defined(GPS_SERIAL)
    uint8_t c = SerialAvailable(GPS_SERIAL);
    while (c--) {
      if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
    #elif defined(TINY_GPS)
    {
      {
      tinygps_query();
    #elif defined(GPS_FROM_OSD)
    {
      if(GPS_update & 2) {  // Once second bit of GPS_update is set, indicate new GPS datas is readed from OSD - all in right format.
        GPS_update &= 1;    // We have: GPS_fix(0-2), GPS_numSat(0-15), GPS_coord[LAT & LON](signed, in 1/10 000 000 degres), GPS_altitude(signed, in meters) and GPS_speed(in cm/s)                     
    #endif
       if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
       
       //If we have a GPS Fix and the Number of Satellites are larger than or equal to 5 we have enough data to set home.
       //Home is constantly reset, tracker can be used for in stationary mode or moving mode and it has no effect on home.
        if (f.GPS_FIX && GPS_numSat >= 5) {
            GPS_reset_home_position();

          //Apply moving average filter to GPS data
          #if defined(GPS_FILTERING)
            GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
            
            //Get GPS data for both Axis (lat/long) and average them.
            for (axis = 0; axis< 2; axis++) {
              GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
              GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
            }
          #endif

          //calculate distance and bearings for gui and other stuff continously - From home to copter
          uint32_t dist;
          int32_t  dir;
          GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
          GPS_distanceToHome = dist/100;
          GPS_directionToHome = dir/100;
          
          //If we don't have home set, do not display anything
          if (!f.GPS_FIX_HOME) {
             GPS_distanceToHome = 0;
             GPS_directionToHome = 0;
          }
          
          //calculate the current velocity based on gps coordinates continously to get a valid speed from the moment HOME is set
          GPS_calc_velocity();        
        }
      }
    }
  #endif
}

void GPS_reset_home_position() {
      GPS_home[LAT] = GPS_coord[LAT];
      GPS_home[LON] = GPS_coord[LON];
      GPS_calc_longitude_scaling(GPS_coord[LAT]);  //need an initial value for distance and bearing calc
      
    //Set ground altitude
    f.GPS_FIX_HOME = 1;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}

// This code is used for parsing NMEA data
#if defined(GPS_SERIAL)

// It seems the below splits the decimal LAT and LONG into two different pieces
#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
}

//Staging Function before sending to actual parsing function for each GPS output type.
bool GPS_newFrame(char c) {
  #if defined(NMEA)
    return GPS_NMEA_newFrame(c);
  #endif
  #if defined(UBLOX)
    return GPS_UBLOX_newFrame(c);
  #endif
  #if defined(MTK_BINARY16) || defined(MTK_BINARY19)
    return GPS_MTK_newFrame(c);
  #endif
}


#if defined(NMEA)
  /* This is a light implementation of a GPS frame decoding
     This should work with most of modern GPS devices configured to output NMEA frames.
     It assumes there are some NMEA GGA frames to decode on the serial bus
     Here we use only the following data :
       - latitude
       - longitude
       - GPS fix is/is not ok
       - GPS num sat (4 is enough to be +/- reliable)
       - GPS altitude
       - GPS speed
  */
  #define FRAME_GGA  1
  #define FRAME_RMC  2
  
  bool GPS_NMEA_newFrame(char c) {
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, frame = 0;
  
    if (c == '$') {
      param = 0; offset = 0; parity = 0;
    } else if (c == ',' || c == '*') {
      string[offset] = 0;
      if (param == 0) { //frame identification
        frame = 0;
        if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
        if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
      } else if (frame == FRAME_GGA) {
        if      (param == 2)                     {GPS_coord[LAT] = GPS_coord_to_degrees(string);}
        else if (param == 3 && string[0] == 'S') GPS_coord[LAT] = -GPS_coord[LAT];
        else if (param == 4)                     {GPS_coord[LON] = GPS_coord_to_degrees(string);}
        else if (param == 5 && string[0] == 'W') GPS_coord[LON] = -GPS_coord[LON];
        else if (param == 6)                     {f.GPS_FIX = (string[0]  > '0');}
        else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
        else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}  // altitude in meters added by Mis
      } else if (frame == FRAME_RMC) {
        if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
        else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
      }
      param++; offset = 0;
      if (c == '*') checksum_param=1;
      else parity ^= c;
    } else if (c == '\r' || c == '\n') {
      if (checksum_param) { //parity checksum
        uint8_t checksum = hex_c(string[0]);
        checksum <<= 4;
        checksum += hex_c(string[1]);
        if (checksum == parity) frameOK = 1;
      }
      checksum_param=0;
    } else {
       if (offset < 15) string[offset++] = c;
       if (!checksum_param) parity ^= c;
    }
    if (frame) GPS_Present = 1;
    return frameOK && (frame==FRAME_GGA);
  }
#endif //NMEA

#endif //SERIAL GPS

#endif // GPS
