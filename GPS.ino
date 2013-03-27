// I wanted to get a result so I decided to use the TinyGPS library.
// At some stage I will write the code to do the GPS code natively.

void getgps(TinyGPS &gps)
{

  gps.f_get_position(&trac_lat, &trac_lon); 

  trac_sats = gps.satellites();

}
