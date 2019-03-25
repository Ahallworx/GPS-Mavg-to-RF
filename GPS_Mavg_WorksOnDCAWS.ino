// Add the GPS library from Adafruit
#include <Adafruit_GPS.h>  //Note:Put between double quotes "Adafruit_GPS.h" if not in the arduino main folder and is defined locally.

// This sketch is ONLY for the Arduino DUE
// For some reason the DUE does not support interrupts
// originating from the GPS at this point.
// Thus we must use the serial communication and a timer.
//   GPS TX to Arduino Due Serial1 RX pin 16
//   GPS RX to Arduino Due Serial1 TX pin 17
// The communication with the GPS chip will
// use that serial port

#define GPS_Serial Serial3
Adafruit_GPS GPS(&GPS_Serial);   //Create  GPS and point to Serial2

void setup()  
{  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS
  GPS.begin(9600);
  GPS_Serial.begin(9600);

  
  // Serial monitor set to 9600 baud
  Serial.begin(9600);
  
  // Request RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate (1 Hz)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  // Request updates on antenna status
  GPS.sendCommand(PGCMD_ANTENNA);

  // Wait for one second
  delay(1000);
}

// This will read the hardware timer value (in ms) when the program starts
uint32_t timer = millis();

// Declare the moving average window size, latitude/longitude buffers and initialization flag
const int winsz = 100;
double latbuffer[winsz], lonbuffer[winsz];
boolean avginit = true;

void loop()
{
  // Declare the averaged lat/lon as local variables (8 bytes, double precision!!!)
  double avglat, avglon;
          
  // Read one character at a time from the GPS in the main loop
  char c = GPS.read();
    
  // if a complete GPS message is received, check the checksum and parse it
  if (GPS.newNMEAreceived())  {
    // if we fail to parse a complete GPS message, just wait for another
    // and set the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // The timer goes back to 0 after 50 days
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
  
  // Every second, print the current lat/long
  if (millis() - timer > 1000){
    // reset the timer
    timer = millis();
Serial.println("here 1");
    if((GPS.latitude != 0.0) && (GPS.longitude != 0.0)) {
      // Display the lat/lon in the Serial Monitor window with full resolution
      // DDMM.FFFFFF (DD: degree, MM: minute, FFFFFF: fraction of minute)      
      Serial.print(GPS.latitude,6);
      Serial.print(", ");
      Serial.print(GPS.longitude,6);
      Serial.print(",");                                        //changed to "," from "\n".!!!!
        
      // Calculate the moving average and set the init flag to false
      mvavg((double) GPS.latitude,(double) GPS.longitude,avginit,&avglat,&avglon);
      avginit = false;
      // Display the averaged lat/lon in the Serial Monitor window with full resolution
      // DDMM.FFFFFF (DD: degree, MM: minute, FFFFFF: fraction of minute)  
      Serial.print(avglat,6);
      Serial.print(", "); 
      Serial.print(avglon,6);
      Serial.print("\n");           
    }
  }
}

// 64-bit double-precision moving average
// inputs:  latest latitude (double), init (boolean)
// output:  latitude and longitude moving average (double *)
void mvavg(double lat, double lon, boolean mvinit, double *latmean, double *lonmean)
{
  // These static variables are kept in memory at all time
  // and can only be used by this function
  static double latsum, lonsum;     // cumulated latitudes/longitudes
  static int n;                     // actual window size
  static int k;                     // circular buffer index

  // if this is the first call, cumulated lat value = input lat/lon value
  // and window size = 1; buffer the values
  if (mvinit == true) {
    latsum = lat;
    lonsum = lon;
    n = 1;
    k = 0;
    latbuffer[k] = lat;
    lonbuffer[k] = lon;    
  }
  // if this is not the first call
  else{
    // if we have collected less points than the window size
    // cumulate the value and increment the window size
    if(n<winsz) {
      latsum += lat;
      latbuffer[++k] = lat;
      lonsum += lon;
      lonbuffer[k] = lon;
      n++;
    }
    
    // if we have already collected more points than the window
    // size, add the latest record and remove the oldest one
    // we also replace the oldest record in the circular buffer
    // with the new one
    else {
      k++;
      k %= winsz;
      latsum += (lat-latbuffer[k]);
      latbuffer[k] = lat;
      lonsum += (lon-lonbuffer[k]);
      lonbuffer[k] = lon;
    }
  }
  
  // Calculate and return the moving average
  *latmean = latsum/(double)n;
  *lonmean = lonsum/(double)n;
}
