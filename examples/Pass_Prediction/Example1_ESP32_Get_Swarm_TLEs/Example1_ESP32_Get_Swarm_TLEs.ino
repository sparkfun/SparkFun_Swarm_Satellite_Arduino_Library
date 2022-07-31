/*
  Use ESP32 WiFi to get the Two-Line Elements for the Swarm satellites
  By: SparkFun Electronics / Paul Clark
  Date: July 31st, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  ** If you have enjoyed this code, please consider making a donation to CelesTrak: https://celestrak.org/ **

  This example shows how to:
    * Download the list of Two-Line Elements (orbit parameters) for _all_ of the Swarm satellites from CelesTrak
      and save it to microSD card
    * Download the satellite pass prediction for your location from the Swarm server and save it to microSD card
    * Cross-correlate the two data sets to build a list of the individual satellites that cover your location
  
  You can then use the TLEs and the satellite list to predict the next satellite pass for your location - offline!
  
  Please see the next examples for details: Example2_ESP32_Get_My_Swarm_TLEs and Example3_ESP32_Predict_Next_Swarm_Pass
  
  This example has a lot of calculations and file accessing to do. It is _very_ slow... But you should only need to run it
  occasionally to update the list of satellites for your location. Example2 downloads only the TLEs for your location and
  is much faster. Example3 uses the TLEs downloaded by Example2 - not the full set.

  Update secrets.h with your:
  - WiFi credentials

  Update July 31st, 2022: the geospatial location is requested from the modem. Uncomment #define noModemGeospatial below to
  specify a different location.

  This example is written for the SparkFun Thing Plus C but can be adapted for any ESP32 board.

  If the SD card is not detected ("Card Mount Failed"), try adding a 10K pull-up resistor between 19/POCI and 3V3.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus C - ESP32 WROOM

*/

//#define noModemGeospatial // Uncomment this line if you want to define your own location for the pass predictions

#ifdef noModemGeospatial

const char myLatitude[] =   "55.000";     //                  <-- Update this with your latitude if desired
const char myLongitude[] =  "-1.000";     //                  <-- Update this with your longitude if desired
const char myAltitude[] =   "100";        //                  <-- Update this with your altitude in m if desired

#else

#include <SparkFun_Swarm_Satellite_Arduino_Library.h> //Click here to get the library:  http://librarymanager/All#SparkFun_Swarm_Satellite

SWARM_M138 mySwarm;

#if defined(ARDUINO_ESP32_DEV)
// If you are using the ESP32 Dev Module board definition, you need to create the HardwareSerial manually:
#pragma message "Using HardwareSerial for M138 communication - on ESP32 Dev Module"
HardwareSerial swarmSerial(2); //TX on 17, RX on 16
#else
// Serial1 is supported by the new SparkFun ESP32 Thing Plus C board definition
#pragma message "Using Serial1 for M138 communication"
#define swarmSerial Serial1 // Use Serial1 to communicate with the modem. Change this if required.
#endif

// If you are using the Swarm Satellite Transceiver MicroMod Function Board:
//
// The Function Board has an onboard power switch which controls the power to the modem.
// The power is disabled by default.
// To enable the power, you need to pull the correct PWR_EN pin high.
//
// Uncomment and adapt a line to match your Main Board and Processor configuration:
//#define swarmPowerEnablePin A1 // MicroMod Main Board Single (DEV-18575) : with a Processor Board that supports A1 as an output
//#define swarmPowerEnablePin 39 // MicroMod Main Board Single (DEV-18575) : with e.g. the Teensy Processor Board using pin 39 (SDIO_DATA2) to control the power
//#define swarmPowerEnablePin 4  // MicroMod Main Board Single (DEV-18575) : with e.g. the Artemis Processor Board using pin 4 (SDIO_DATA2) to control the power
//#define swarmPowerEnablePin G5 // MicroMod Main Board Double (DEV-18576) : Slot 0 with the ALT_PWR_EN0 set to G5<->PWR_EN0
//#define swarmPowerEnablePin G6 // MicroMod Main Board Double (DEV-18576) : Slot 1 with the ALT_PWR_EN1 set to G6<->PWR_EN1

#endif

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define sd_cs SS // microSD chip select - this should work on most boards
//const int sd_cs = 5; //Uncomment this line to define a specific pin for the chip select (e.g. pin 5 on the Thing Plus C)

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <Sgp4.h> //Click here to get the library: http://librarymanager/All#SparkFun_SGP4_Arduino_Library

Sgp4 sat;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// The Two-Line Element orbit data will be downloaded from CelesTrak
// https://celestrak.org/NORAD/elements/gp.php?GROUP=swarm&FORMAT=tle

const char celestrakServer[] = "https://celestrak.org";

const char getSwarmTLE[] = "NORAD/elements/gp.php?GROUP=swarm&FORMAT=tle";

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// The Swarm Pass-Checker data can be downloaded from their server

const char swarmPassCheckerServer[] = "https://bumblebee.hive.swarm.space";

const char passCheckerAPI[] = "api/v1/passes?";

const char latPrefix[] = "lat=";
const char lonPrefix[] = "&lon=";
const char altPrefix[] = "&alt=";
const char mergeSuffix[] = "&merge=false";

// The pass checker data is returned in non-pretty JSON format:
#define startOfFirstStartPass 32  // 8000\r\n{"passes":[{"start_pass":"YYYY-MM-DDTHH:MM:SSZ
                                  // ----- - --------------------------^
#define startPassDateTimeLength 19
#define endOffset 15 // Offset from the start_pass "Z" to the start of the end_pass
#define elevationOffset 41 // Offset from the end_pass "Z" to the start of the max_elevation
#define startOffset 12 // Offset from the "s" of start_pass to the start of the year

// At the time or writing:
//  Swarm satellites are named: SPACEBEE-n or SPACEBEENZ-n
//  SPACEBEE numbers are 1 - 155 (8 and 9 are missing)
//  SPACEBEENZ numbers are 1 - 22
const int maxSats = 176;

// Stop checking when we find this many satellite duplications for a single satellite
// (When > 24 hours of passes have been processed)
#define satPassLimit 7

// Ignore any false positives (satellites with fewer than this many passes)
#define satPassFloor 2

// Check for a match: start_pass +/- 2.5 seconds (in Julian Days)
const double predictionStartError = 0.000029;
// Check for a match: end_pass +/- 2.5 seconds (in Julian Days)
const double predictionEndError = 0.000029;
// Check for a match on max_elevation. Accuracy is worst at Zenith, much better towards the horizon.
const double maxElevationError = 2.0;
const double elevationZenith = 70.0;
const double maxElevationErrorZenith = 5.0;

#define nzOffset 100000 // Add this to the satellite number to indicate SPACEBEENZ

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  // Swarm Satellite Transceiver MicroMod Function Board PWR_EN
  #ifdef swarmPowerEnablePin
  pinMode(swarmPowerEnablePin, OUTPUT); // Enable modem power 
  digitalWrite(swarmPowerEnablePin, HIGH);
  #endif

  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("Example : Swarm Two-Line Elements for your location"));

  while (Serial.available()) Serial.read(); // Empty the serial buffer
  Serial.println(F("Press any key to begin..."));
  while (!Serial.available()); // Wait for a keypress
  Serial.println();

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Wait for the modem to get a GPS fix - if desired
  
#ifndef noModemGeospatial
  //mySwarm.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  bool modemBegun = mySwarm.begin(swarmSerial); // Begin communication with the modem
  
  while (!modemBegun) // If the begin failed, keep trying to begin communication with the modem
  {
    Serial.println(F("Could not communicate with the modem. It may still be booting..."));
    delay(2000);
    modemBegun = mySwarm.begin(swarmSerial);
  }

  // Call getGeospatialInfo to request the most recent geospatial information
  Swarm_M138_GeospatialData_t info;
  
  Swarm_M138_Error_e err = mySwarm.getGeospatialInfo(&info);
  
  while (err != SWARM_M138_SUCCESS)
  {
    Serial.print(F("Swarm communication error: "));
    Serial.print((int)err);
    Serial.print(F(" : "));
    Serial.println(mySwarm.modemErrorString(err)); // Convert the error into printable text
    Serial.println(F("The modem may not have acquired a valid GPS fix..."));
    delay(2000);
    err = mySwarm.getGeospatialInfo(&info);
  }

  Serial.print(F("getGeospatialInfo returned: "));
  Serial.print(info.lat, 4);
  Serial.print(F(","));
  Serial.print(info.lon, 4);
  Serial.print(F(","));
  Serial.println(info.alt);
#endif

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Connect to WiFi.

  Serial.print(F("Connecting to local WiFi"));

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected!"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Open the file for the TLE data so we can stream the HTTP GET straight to file

  if (!SD.begin(sd_cs)) {
    Serial.println("Card Mount Failed! Freezing...");
    while (1)
      ;
  }

  File tleFile = SD.open("/swarmTLE.txt", FILE_WRITE);
  if (!tleFile)
  {
    Serial.println(F("Could not open swarmTLE.txt for writing! Freezing..."));
    while (1)
      ;
  }  

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Use HTTP GET to receive the TLE data

  const int URL_BUFFER_SIZE  = 256;
  char theURL[URL_BUFFER_SIZE]; // This will contain the HTTP URL
  int payloadSize = 0; // This will be updated with the length of the data we get from the server
  String payload; // This will store the data we get from the server
  HTTPClient http;
  int httpCode;

  // Assemble the URL
  // Note the slash after the first %s (celestrakServer)
  snprintf(theURL, URL_BUFFER_SIZE, "%s/%s",
    celestrakServer,
    getSwarmTLE);

  Serial.println(F("Requesting the Swarm TLE data from CelesTrak"));
  Serial.print(F("HTTP URL is: "));
  Serial.println(theURL);

  http.begin(theURL);

  httpCode = http.GET(); // HTTP GET

  // httpCode will be negative on error
  if(httpCode > 0)
  {
    // HTTP header has been sent and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\r\n", httpCode);
  
    // If the GET was successful, read the data
    if(httpCode == HTTP_CODE_OK) // Check for code 200
    {
      // Code taken from the ESP32 ReuseConnection example. Mostly...
      
      payloadSize = http.getSize();
      Serial.printf("Server returned %d bytes\r\n", payloadSize);
      if (payloadSize == -1)
        Serial.println(F("(That is OK. It means there is no Content-Length header.)"));
      Serial.print(F("Reading data"));

      // Create a buffer for the bytes
      uint8_t *content = new uint8_t[512]; // Read up to 512 bytes at a time (SD write chunk)
      *content = 0; // Clear the first byte

      // Get TCP stream
      WiFiClient *stream = http.getStreamPtr();

      unsigned long dataArrival = millis(); // Timeout

      // Read all the data from the server
      while ((http.connected())
              && ((payloadSize > 0) || (payloadSize == -1))
              && (millis() < (dataArrival + 5000))) // Timeout after 5 seconds
      {
        // Get available data size
        size_t avail = stream->available();

        if (avail)
        {
          // Read up to 512 bytes
          int howMany = stream->readBytes(content, ((avail > 512) ? 512 : avail));

          // Write it to file
          tleFile.write(content, howMany);

          if (payloadSize > 0)
            payloadSize -= howMany;

          if (millis() > (dataArrival + 1000))
          {
            Serial.print(F(".")); // Print a dot every second
            dataArrival = millis();
          }
        }
        delay(1);
      }

      Serial.println();

      delete[] content; // Deallocate the buffer
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\r\n", http.errorToString(httpCode).c_str());
  }

  http.end();

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Close the TLE file

  tleFile.close();

  Serial.println(F("The Swarm TLE data has been saved to SD card"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Open the file for the Swarm Pass-Checker data so we can stream the HTTP GET straight to file

  File ppFile = SD.open("/swarmPP.txt", FILE_WRITE);
  if (!ppFile)
  {
    Serial.println(F("Could not open swarmPP.txt for writing! Freezing..."));
    while (1)
      ;
  }  

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Use HTTP GET to receive the Swarm pass prediction for your location

  payload = ""; // Clear the existing payload
  payloadSize = 0; // This will be updated with the length of the data we get from the server

#ifndef noModemGeospatial
  char myLatitude[12];
  sprintf(myLatitude, "%.4f", info.lat);
  char myLongitude[12];
  sprintf(myLongitude, "%.4f", info.lon);
  char myAltitude[12];
  sprintf(myAltitude, "%.2f", info.alt);
#endif

  // Assemble the URL
  // Note the slash after the first %s
  snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s%s%s",
    swarmPassCheckerServer,
    passCheckerAPI,
    latPrefix,
    myLatitude,
    lonPrefix,
    myLongitude,
    altPrefix,
    myAltitude,
    mergeSuffix);

  Serial.println(F("Requesting the Swarm pass-checker data"));
  Serial.print(F("HTTP URL is: "));
  Serial.println(theURL);

  HTTPClient http2;
  http2.begin(theURL);

  httpCode = http2.GET();

  // httpCode will be negative on error
  if(httpCode > 0)
  {
    // HTTP header has been sent and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\r\n", httpCode);
  
    // If the GET was successful, read the data
    if(httpCode == HTTP_CODE_OK) // Check for code 200
    {
      // Code taken from the ESP32 ReuseConnection example. Mostly...
      
      payloadSize = http2.getSize();
      Serial.printf("Server returned %d bytes\r\n", payloadSize);
      if (payloadSize == -1)
        Serial.println(F("(That is OK. It means there is no Content-Length header.)"));
      Serial.print(F("Reading data"));

      // Create a buffer for the bytes
      uint8_t *content = new uint8_t[512]; // Read up to 512 bytes at a time (SD write chunk)
      *content = 0; // Clear the first byte

      // Get TCP stream
      WiFiClient *stream2 = http2.getStreamPtr();

      unsigned long dataArrival = millis(); // Timeout

      // Read all the data from the server
      while ((http2.connected())
              && ((payloadSize > 0) || (payloadSize == -1))
              && (millis() < (dataArrival + 5000))) // Timeout after 5 seconds
      {
        // Get available data size
        size_t avail = stream2->available();

        if (avail)
        {
          // Read up to 512 bytes
          int howMany = stream2->readBytes(content, ((avail > 512) ? 512 : avail));

          // Write it to file
          ppFile.write(content, howMany);

          if (payloadSize > 0)
            payloadSize -= howMany;

          if (millis() > (dataArrival + 1000))
          {
            Serial.print(F(".")); // Print a dot every second
            dataArrival = millis();
          }
        }
        delay(1);
      }

      Serial.println();

      delete[] content; // Deallocate the buffer
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\r\n", http.errorToString(httpCode).c_str());
  }

  http2.end();
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Close the file

  ppFile.close();

  Serial.println(F("The Swarm pass-checker data has been saved to SD card"));

  listDir(SD, "/", 0); // List the card directory to see if the files were written correctly

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Disconnect the WiFi as it's no longer needed

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println();
  Serial.println(F("WiFi disconnected"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Set site latitude[°], longitude[°] and altitude[m]
  
  float myLat, myLon, myAlt;
  sscanf(myLatitude, "%f", &myLat);
  sscanf(myLongitude, "%f", &myLon);
  sscanf(myAltitude, "%f", &myAlt);
  sat.site((double)myLat, (double)myLon, (double)myAlt);        

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Record which satellites have been cross-correlated

  int foundNumSats = 0;
  unsigned long foundSats[maxSats];
  int satPassCount[maxSats];
  for (int i = 0; i < maxSats; i++)
  {
    foundSats[i] = 0;
    satPassCount[i] = 0;
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Keep going until we hit the end of the pass prediction file
  // Or satPassCount reaches satPassLimit

  bool keepGoing = true;

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Open the pass prediction file for reading

  ppFile = SD.open("/swarmPP.txt", FILE_READ);
  if (!ppFile) {
    Serial.println(F("Pass Prediction File Open Failed! Freezing..."));
    while (1)
      ;
  }

  char fileChar;
  for (int i = 0; i < startOfFirstStartPass; i++) // Point at the first start_pass
  {
    if (ppFile.readBytes(&fileChar, 1) != 1)
      keepGoing = false;
  }

  Serial.println();
  Serial.println(F("Starting the pass prediction calculations. This will take a LONG time! Go make a cup of tea..."));
  Serial.println();
  Serial.println(F("(No match will be found for satellites already above the horizon)"));
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Step through the pass prediction file, extract the start_pass and max_elevation
  //
  // Keep going until we hit the end of the file

  while (keepGoing)
  {

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Extract the start_pass

    char startPass[startPassDateTimeLength + 1];
    startPass[startPassDateTimeLength] = 0; // Null-terminate the dateTime
    if (ppFile.readBytes((char *)&startPass, startPassDateTimeLength) != startPassDateTimeLength)
    {
      keepGoing = false;
      break;
    }

    Serial.println();
    Serial.print(F("Extracted start_pass: "));
    Serial.print(startPass);

    // Convert it to Julian Day
    int year, month, day, hour, minute, second;
    if (sscanf(startPass, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second) != 6)
    {
      keepGoing = false;
      break;      
    }
    double predictionStart;
    jday(year, month, day, hour, minute, second, 0, false, predictionStart);

    Serial.print(F("  Julian Day: "));
    Serial.print(predictionStart, 5);

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Extract the end_pass

    for (int i = 0; i < endOffset; i++) // Point at the end_pass
    {
      if (ppFile.readBytes(&fileChar, 1) != 1)
      {
        keepGoing = false;
      }
    }

    if (!keepGoing) // Exit the while loop if we hit the end of the file
    {
      Serial.println();
      break;
    }

    char endPass[startPassDateTimeLength + 1];
    endPass[startPassDateTimeLength] = 0; // Null-terminate the dateTime
    if (ppFile.readBytes((char *)&endPass, startPassDateTimeLength) != startPassDateTimeLength)
    {
      keepGoing = false;
      break;
    }

    //Serial.print(F("  end_pass: "));
    //Serial.print(endPass);

    // Convert it to Julian Day
    if (sscanf(endPass, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second) != 6)
    {
      keepGoing = false;
      break;      
    }
    double predictionEnd;
    jday(year, month, day, hour, minute, second, 0, false, predictionEnd);

    Serial.print(F("  end_pass Julian Day: "));
    Serial.print(predictionEnd, 5);

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Extract the max_elevation

    for (int i = 0; i < elevationOffset; i++) // Point at the max_elevation
    {
      if (ppFile.readBytes(&fileChar, 1) != 1)
      {
        keepGoing = false;
      }
    }

    if (!keepGoing) // Exit the while loop if we hit the end of the file
    {
      Serial.println();
      break;
    }

    char maxElevationStr[7];
    maxElevationStr[6] = 0; // Null-terminate the max_elevation

    if (ppFile.readBytes((char *)&maxElevationStr, 6) != 6) // Read 3 decimal places (nn.nnn)
    {
      keepGoing = false;
      Serial.println();
      break;
    }

    float maxElevation;
    if (sscanf(maxElevationStr, "%f", &maxElevation) != 1)
    {
      keepGoing = false;
      Serial.println();
      break;
    }

    Serial.print(F("  max_elevation: "));
    Serial.println(maxElevation, 3);

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Step through the TLE file.
    // For each satellite, perform our own pass prediction using our location and predictionStart.
    // Record if we have a match.

    tleFile = SD.open("/swarmTLE.txt", FILE_READ);
    if (!tleFile) {
      Serial.println(F("TLE File Open Failed! Freezing..."));
      while (1)
        ;
    }

    bool keepReadingTLEs = true;

    while (keepReadingTLEs)
    {
  
      // Allocate 30 bytes to store the satellite name
      char satelliteName[30];
      for (int i = 0; i < 30; i++)
        satelliteName[i] = 0;
  
      // Read the satellite name
      int satNameLength = tleFile.readBytesUntil('\n', (char *)satelliteName, 30);
      satelliteName[satNameLength] = 0; // Null-terminate the name
      //Serial.print(F("Performing prediction for satellite "));
      //Serial.println(satelliteName);
  
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
        keepReadingTLEs = false;
      }
      else
      {
        sat.init(satelliteName, lineOne, lineTwo); //initialize satellite parameters     

        double overpassStart = 0.0, overpassEnd = 0.0, overpassMaxElev = 0.0;
        double maxElevationDbl = (double)maxElevation;

        // Store the last 10 predictions to try and avoid duplicates
        static unsigned long lastSatNums[10] = {0,0,0,0,0,0,0,0,0,0};
        static double lastOverpassStarts[10] = {0,0,0,0,0,0,0,0,0,0};

        // Calculate the next overpass
        // Start one minute early 
        if (Predict(predictionStart - (1.0 / (24.0 * 60.0)), &overpassStart, &overpassEnd, &overpassMaxElev))
        {
          double maxElevErr = (overpassMaxElev > elevationZenith) ? maxElevationErrorZenith : maxElevationError;

          // Convert the satellite number to integer
          unsigned long satNum = 0;
          if (satelliteName[8] == '-') // Look for SPACEBEE-nnn
          {
            satNum = satelliteName[9] - '0'; // Extract the satellite number (max 99999)
            if ((satelliteName[10] >= '0') && (satelliteName[10] <= '9'))
            {
              satNum *= 10;
              satNum += satelliteName[10] - '0';
              if ((satelliteName[11] >= '0') && (satelliteName[11] <= '9'))
              {
                satNum *= 10;
                satNum += satelliteName[11] - '0';
                if ((satelliteName[12] >= '0') && (satelliteName[12] <= '9'))
                {
                  satNum *= 10;
                  satNum += satelliteName[12] - '0';
                  if ((satelliteName[13] >= '0') && (satelliteName[13] <= '9'))
                  {
                    satNum *= 10;
                    satNum += satelliteName[13] - '0';
                  }
                }
              }
            }
          }
          else if ((satelliteName[8] == 'N') && (satelliteName[9] == 'Z')) // Look for SPACEBEENZ-nnn
          {
            // Add nzOffset to the SPACEBEENZ numbers when recording them
            satNum = nzOffset + satelliteName[11] - '0'; // Extract the satellite number (max 99999)
            if ((satelliteName[12] >= '0') && (satelliteName[12] <= '9'))
            {
              satNum *= 10;
              satNum += satelliteName[12] - '0';
              if ((satelliteName[13] >= '0') && (satelliteName[13] <= '9'))
              {
                satNum *= 10;
                satNum += satelliteName[13] - '0';
                if ((satelliteName[14] >= '0') && (satelliteName[14] <= '9'))
                {
                  satNum *= 10;
                  satNum += satelliteName[14] - '0';
                  if ((satelliteName[15] >= '0') && (satelliteName[15] <= '9'))
                  {
                    satNum *= 10;
                    satNum += satelliteName[15] - '0';
                  }
                }
              }
            }
          }

          bool duplicatePass = false;
          for (int i = 0; i < 10; i++)
          {
            duplicatePass |= ((lastSatNums[i] == satNum) && (overpassStart >= lastOverpassStarts[i]) && (overpassStart <= (lastOverpassStarts[i] + predictionStartError)));
          }
          
          if  ((overpassStart < (predictionStart + predictionStartError)) // Check for a match
            && (overpassStart > (predictionStart - predictionStartError))
            && (overpassEnd < (predictionEnd + predictionEndError))
            && (overpassEnd > (predictionEnd - predictionEndError))
            && (overpassMaxElev < (maxElevationDbl + maxElevErr))
            && (overpassMaxElev > (maxElevationDbl - maxElevErr)))
          {
            if (duplicatePass)
              Serial.println(F("Probable duplicate pass. Skipping..."));
            else
            {
              Serial.print(F("Pass match found for satellite: "));
              Serial.println((char *)satelliteName);
  
              Serial.print(F("Prediction result was: start_pass Julian Day: "));
              Serial.print(overpassStart, 5);
              Serial.print(F("  end_pass Julian Day: "));
              Serial.print(overpassEnd, 5);
              Serial.print(F("  max_elevation: "));
              Serial.println(overpassMaxElev, 3);
            
              // Record the match
              if (satNum > 0)
              {
                bool foundAgain = false;
                for (int i = 0; i < foundNumSats; i++)
                {
                  if (foundSats[i] == satNum) // Have we found this satellite before?
                  {
                    foundAgain = true;
                    satPassCount[i] = satPassCount[i] + 1;
                  }
                }
                if ((!foundAgain) && (foundNumSats < maxSats))
                {
                  foundSats[foundNumSats] = satNum;
                  satPassCount[foundNumSats++] = 1;
                }
                keepReadingTLEs = false; // Stop searching after one match
              }
            }
            for (int i = 0; i < 9; i++)
            {
              lastSatNums[i] = lastSatNums[i+1];
              lastOverpassStarts[i] = lastOverpassStarts[i+1];
            }
            lastSatNums[9] = satNum;
            lastOverpassStarts[9] = overpassStart;
          }
        }
      }
    }
    
    tleFile.close(); // Close the TLE file

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Point at the next start_pass

    bool sFound = false;
    while (!sFound && keepGoing) // Find the next "s"tart_pass 
    {
      if (ppFile.readBytes(&fileChar, 1) != 1)
        keepGoing = false;
      if (fileChar == 's')
        sFound = true;
    }
    if (sFound)
    {
      for (int i = 0; i < startOffset; i++) // Point at the year
      {
        if (ppFile.readBytes(&fileChar, 1) != 1)
          keepGoing = false;
      }
    }

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Check if satPassCount has reached satPassLimit

    for (int i = 0; i < foundNumSats; i++)
    {
      if (satPassCount[i] >= satPassLimit)
      {
        Serial.println();
        Serial.print(F("Found "));
        Serial.print(satPassCount[i]);
        Serial.println(F(" duplicate satellite passes. Exiting..."));
        Serial.println();
        keepGoing = false;
      }
    }
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Close the pass prediction file

  ppFile.close();

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Finally write all the found satellites to mySwarm.txt

  Serial.println(F("Writing the satellite names to mySwarm.txt"));

  File file = SD.open("/mySwarm.txt", FILE_WRITE);
  if (file)
  {
    for (int i = 0; i < foundNumSats; i++)
    {
      if (satPassCount[i] >= satPassFloor) // Make sure the satellite has been seen at least satPassFloor times
      {
        if (foundSats[i] > nzOffset)
        {
          file.print("SPACEBEENZ-");
          file.println(foundSats[i] - nzOffset);
          Serial.print(F("SPACEBEENZ-"));
          Serial.print(foundSats[i] - nzOffset);
          Serial.print(F("\t("));
          Serial.print(satPassCount[i]);
          Serial.println(F(" passes)"));
        }
        else if (foundSats[i] > 0)
        {
          file.print("SPACEBEE-");
          file.println(foundSats[i]);
          Serial.print(F("SPACEBEE-"));
          Serial.print(foundSats[i]);
          Serial.print(F("\t("));
          Serial.print(satPassCount[i]);
          Serial.println(F(" passes)"));
        }
      }
    }
    Serial.println();
  }
  file.close();

  listDir(SD, "/", 0); // List the card directory to see if the files were written correctly

  Serial.println(F("\r\nAll done!"));
}
  
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  // Nothing to do here
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Predict the next satellite pass.
// Start the prediction at startTimeJD (Julian Day).
// Return the start of the pass in overpassStart (Julian Day).
// Return the end of the pass in overpassEnd (Julian Day).
// Return the maximum elevation in maxElevation.
// Return true if the prediction was successful.
bool Predict(double startTimeJD, double *overpassStart, double *overpassEnd, double *maxElevation)
{
  passinfo overpass;                      //structure to store overpass info
  sat.initpredpoint( startTimeJD , 0.0 ); //finds the startpoint
  
  bool error;
  
  //int year; int mon; int day; int hr; int minute; double sec;
  
  error = sat.nextpass(&overpass, 20, false, 15.0); //search for the next overpass, if there are more than 20 maximums below 15 degrees it returns false
  
  if ( error == 1) //no error
  {
    //invjday(overpass.jdstart, 0, false, year, mon, day, hr, minute, sec); // Time Zone 0 : use UTC
    //Serial.println("  Overpass: " + String(day) + '/' + String(mon) + '/' + String(year));
    //Serial.println("    Start: " + String(hr) + ':' + String(minute) + ':' + String(sec));
    //Serial.println("    Max Elevation = " + String(overpass.maxelevation) + "°");

    *overpassStart = overpass.jdstart;
    *overpassEnd = overpass.jdstop;
    *maxElevation = overpass.maxelevation;
  }
  else
  {
    //Serial.println("  Prediction error");
  }

  return (error == 1);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(F("  DIR : "));
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(F("\tFILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
