// I wanted to get a result so I decided to use the TinyGPS library.
// At some stage I will write the code to do the GPS code natively.
void getGPS()
{
  while (Serial2.available())
  {
    if (gps.encode(Serial2.read())) {
      gps.f_get_position(&trac_lat, &trac_lon);
      trac_sats = gps.satellites();
      // If there are more than 5 Sats we have enough for a 3D fix.
      if (trac_sats > 5){
        gpsStat = 1;
      }
    }
  }
}
