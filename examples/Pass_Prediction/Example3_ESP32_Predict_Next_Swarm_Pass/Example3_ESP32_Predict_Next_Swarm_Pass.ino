/*
  Predict the pass of the next Swarm satellite
  By: SparkFun Electronics / Paul Clark
  Date: July 9th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based extensively on @Hopperpop 's Sgp4Predictor example:
  https://github.com/Hopperpop/Sgp4-Library
  https://github.com/Hopperpop/Sgp4-Library/blob/master/examples/Sgp4Predictor/Sgp4Predictor.ino
  
  This example shows how to predict the pass of the next Swarm satellite.
  
  It uses the CelesTrak Two-Line Element data collected by the previous example: Example2_ESP32_Get_My_Swarm_TLEs
  
  ** If you have enjoyed this code, please consider making a donation to CelesTrak: https://celestrak.org/ **

  This example is written for the SparkFun Thing Plus C (SPX-18018) but can be adapted
  for any ESP32 board.

  Use the Boards Manager to install the "ESP32 Arduino" boards.
  
  Add https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  to your "Preferences \ Additional Boards Manager URLs" if required.
  
  Select "ESP32 Dev Module" as the Board (not "SparkFun ESP32 Thing Plus").

  If the SD card is not detected ("Card Mount Failed"), try adding a pull-up resistor between
  19/CIPO and 3V3. A resistor in the range 10K - 1M seems to help.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus C - ESP32 WROOM: https://www.sparkfun.com/products/18018

*/

#include <Sgp4.h> //Click here to get the library: http://librarymanager/All#SparkFun_SGP4_Arduino_Library

Sgp4 sat;

unsigned long unixTime = 1657324800; //                   <-- Update this with the current Unix time

// See https://www.unixtimestamp.com/
// 1657324800 = Sat Jul 09 2022 00:00:00 GMT+0000

double myLat = 55.000; // Latitude 55 degrees North       <-- Update this with your latitude
double myLon = -1.000; // Longitude 1 degree West         <-- Update this with your longitude
double myAlt = 100.00; // Altitude 100m Above Sea Level   <-- Update this with your altitude

// Set the minimum satellite elevation (above the horizon)
// Passes lower than this are ignored
double minimumElevation = 15.0; //                        <-- Update this if required

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define sd_cs SS // microSD chip select - this should work on most boards
//const int sd_cs = 5; //Uncomment this line to define a specific pin for the chip select (e.g. pin 5 on the Thing Plus C)

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{

  delay(1000);

  Serial.begin(115200);
  Serial.println(F("Example : predict next Swarm pass"));

  while (Serial.available()) Serial.read(); // Empty the serial buffer
  Serial.println(F("Press any key to begin..."));
  while (!Serial.available()); // Wait for a keypress
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Set site latitude[°], longitude[°] and altitude[m]

  sat.site(myLat, myLon, myAlt);
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin the SD card and open the file
  
  if (!SD.begin(sd_cs)) {
    Serial.println("Card Mount Failed! Freezing...");
    while (1)
      ;
  }

  File tleFile = SD.open("/mySwmTLE.txt", FILE_READ);

  if (!tleFile) {
    Serial.println("File Open Failed! Freezing...");
    while (1)
      ;
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Read the Two-Line Element for each satellite from tleFile and calculate the next pass

  double nextPassJD = 3650000; // An arbitrary Julian Day way in the future

  bool keepGoing = true;

  while (keepGoing)
  {

    // Allocate 30 bytes to store the satellite name
    char satelliteName[30];

    // Read the satellite name
    int satNameLength = tleFile.readBytesUntil('\n', (char *)satelliteName, 30);
    satelliteName[satNameLength] = 0; // Null-terminate the name

    // Allocate 75 bytes to store the TLE line one
    char lineOne[75];

    // Read line one
    int lineOneLength = tleFile.readBytesUntil('\n', (char *)lineOne, 75);
    lineOne[lineOneLength] = 0; // Null-terminate the line

    // Allocate 75 bytes to store the TLE line two
    char lineTwo[75];

    // Read line two
    int lineTwoLength = tleFile.readBytesUntil('\n', (char *)lineTwo, 75);
    lineTwo[lineTwoLength] = 0; // Null-terminate the line

    // Check we have enough data
    if ((satNameLength < 25) || (lineOneLength < 70) || (lineTwoLength < 70))
    {
      
      Serial.println();
      Serial.println(F("End of mySwmTLE.txt"));
      keepGoing = false;
      
    }
    else
    {
      Serial.println();
      Serial.println("Satellite: " + String(satelliteName));
      Serial.println();

      sat.init(satelliteName, lineOne, lineTwo); //initialize satellite parameters     

      double passStartJD;
      if (Predict(unixTime, &passStartJD)) //Calculates the next overpass
      {
        if (passStartJD < nextPassJD)
        {
          nextPassJD = passStartJD;
        }
      }
    }
  }

  // Close the TLE file
  tleFile.close();

  // Print the next pass
  int year; int mon; int day; int hr; int minute; double sec;
  invjday(nextPassJD, 0, false, year, mon, day, hr, minute, sec); // Time Zone 0 : use UTC
  Serial.println();
  Serial.println("The next satellite pass starts at (UTC): " + String(day) + '/' + String(mon) + '/' + String(year)
                  + " " + String(hr) + ':' + String(minute) + ':' + String(sec));
  
  Serial.println();
  
}


//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  // Nothing to do here
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Predict the next satellite pass.
// Start the prediction at startTime.
// Return the start of the pass in overpassStart.
// Return true if the prediction was successful.
bool Predict(unsigned long startTime, double *overpassStart)
{
  passinfo overpass;                       //structure to store overpass info
  sat.initpredpoint( startTime , 0.0 );    //finds the startpoint. startTime is Unix time (not Julian Day)
  
  bool error;
  int year; int mon; int day; int hr; int minute; double sec;
  
  error = sat.nextpass(&overpass, 20, false, minimumElevation); //search for the next overpass, if there are more than 20 maximums below the minimumElevation it returns false
  
  if (error == 1) //no error, prints overpass information
  {
    invjday(overpass.jdstart, 0, false, year, mon, day, hr, minute, sec); // Time Zone 0 : use UTC
    Serial.println("  Overpass: " + String(day) + '/' + String(mon) + '/' + String(year));
    Serial.println("    Start: az=" + String(overpass.azstart) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));
    
    invjday(overpass.jdmax, 0, false, year, mon, day, hr, minute, sec); // Time Zone 0 : use UTC
    Serial.println("    Max: elev=" + String(overpass.maxelevation) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));
    
    invjday(overpass.jdstop, 0, false, year, mon, day, hr, minute, sec); // Time Zone 0 : use UTC
    Serial.println("    Stop: az=" + String(overpass.azstop) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));  

    *overpassStart = overpass.jdstart;
  }
  else
  {
    Serial.println("  Prediction error");
  }

  return (error == 1);
}
