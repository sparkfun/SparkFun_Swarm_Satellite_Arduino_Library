/*
  Predict the pass of the next Swarm satellite
  By: SparkFun Electronics / Paul Clark
  Date: January 29th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based extensively on @Hopperpop 's Sgp4Predictor example:
  https://github.com/Hopperpop/Sgp4-Library
  https://github.com/Hopperpop/Sgp4-Library/blob/master/examples/Sgp4Predictor/Sgp4Predictor.ino
  
  This example shows how to predict the pass of the next Swarm satellite.
  
  It uses the Two-Line Element data collected by the previous example ("ESP32_Get_Swarm_TLEs").
  
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

#include <Sgp4.h>

Sgp4 sat;

unsigned long unixTime = 1643539800; // Sun Jan 30 2022 10:50:00 UTC

double myLat = 55.000; // Latitude 55 degrees North
double myLon = -1.000; // Longitude 1 degree West
double myAlt = 100.00; // Altitude 100m Above Sea Level

double minimumElevation = 15.0; // Predict passes which have an elevation higher than 15 degrees

int timezone = 0 ;  // Time Zone relative to UTC

int year; int mon; int day; int hr; int minute; double sec;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <FS.h>
#include <SD.h>
#include <SPI.h>

const int sd_cs = 5; //Thing Plus C microSD chip select

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=



void Predict(int many){
    
    passinfo overpass;                       //structure to store overpass info
    sat.initpredpoint( unixTime , 0.0 );     //finds the startpoint

    bool error;

    for (int i = 0; i<many ; i++){
      
        error = sat.nextpass(&overpass, 20, false, minimumElevation); //search for the next overpass, if there are more than 20 maximums below the minimumElevation it returns false
        delay(0);
    
        if ( error == 1){ //no error, prints overpass information
          
          invjday(overpass.jdstart ,timezone ,true , year, mon, day, hr, minute, sec);
          Serial.println("  Overpass: " + String(day) + '/' + String(mon) + '/' + String(year));
          Serial.println("    Start: az=" + String(overpass.azstart) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));
          
          invjday(overpass.jdmax ,timezone ,true , year, mon, day, hr, minute, sec);
          Serial.println("    Max: elev=" + String(overpass.maxelevation) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));
          
          invjday(overpass.jdstop ,timezone ,true , year, mon, day, hr, minute, sec);
          Serial.println("    Stop: az=" + String(overpass.azstop) + "° " + String(hr) + ':' + String(minute) + ':' + String(sec));
          
        }else{
            Serial.println("  Prediction error");
        }
        delay(0);
    }
}


void setup()
{

  delay(1000);

  Serial.begin(115200);
  Serial.println(F("Example : predict Swarm passes"));

  while (Serial.available()) Serial.read(); // Empty the serial buffer
  Serial.println(F("Press any key to begin..."));
  while (!Serial.available()); // Wait for a keypress
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Set site latitude[°], longitude[°] and altitude[m]

  sat.site(myLat, myLon, myAlt);
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin the SD card and open the file
  
  if (!SD.begin(SS)) {
    Serial.println("Card Mount Failed! Freezing...");
    while (1)
      ;
  }

  File file = SD.open("/swarmTLE.txt");

  if (!file) {
    Serial.println("File Open Failed! Freezing...");
    while (1)
      ;
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Read the Two-Line Element for each satellite from file and calculate the next pass

  bool keepGoing = true;

  while (keepGoing)
  {

    // Allocate 30 bytes to store the satellite name
    char satelliteName[30];

    // Read the satellite name
    int satNameLength = file.readBytesUntil('\n', (char *)satelliteName, 30);
    satelliteName[satNameLength] = 0; // Null-terminate the name

    // Allocate 75 bytes to store the TLE line one
    char lineOne[75];

    // Read line one
    int lineOneLength = file.readBytesUntil('\n', (char *)lineOne, 75);
    lineOne[lineOneLength] = 0; // Null-terminate the line

    // Allocate 75 bytes to store the TLE line two
    char lineTwo[75];

    // Read line two
    int lineTwoLength = file.readBytesUntil('\n', (char *)lineTwo, 75);
    lineTwo[lineTwoLength] = 0; // Null-terminate the line

    // Check we have enough data
    if ((satNameLength < 25) || (lineOneLength < 70) || (lineTwoLength < 70))
    {
      
      Serial.println();
      Serial.println(F("End of file..."));
      keepGoing = false;
      
    }
    else
    {
      
      Serial.println();
      Serial.println("Satellite: " + String(satelliteName));
      Serial.println();

      sat.init(satelliteName, lineOne, lineTwo); //initialize satellite parameters     
    
      // Display TLE epoch time
      // Note: this is the epoch time from the TLE, not the unixTime for the prediction
      double jdC = sat.satrec.jdsatepoch;
      invjday(jdC , timezone, true, year, mon, day, hr, minute, sec);
      Serial.println("  TLE Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
      Serial.println();
      
      Predict(1); //Calculates the next overpass
      
    }
  }
  
  file.close();
}


void loop()
{
  // Nothing to do here
}
