// I wanted to get a result so I decided to use the TinyGPS library.
// At some stage I will write the code to do the GPS code natively.
void getGPS()
{
  while (Serial2.available())
  {
    if (gps.encode(Serial2.read())) {
      gps.f_get_position(&trac_lat, &trac_lon);
      trac_sats = gps.satellites();
      // If there are more than 4 sats we have enough for a 3D fix.
      if (trac_sats > 4){
        gpsStat = 1;
      }
      Serial.print("Lattitude: ");Serial.print(trac_lat);Serial.print(" ");Serial.print("Longitude: ");Serial.print(trac_lon);Serial.print(" ");Serial.print("Satellites: ");Serial.println(trac_sats);
      }
  }
}
