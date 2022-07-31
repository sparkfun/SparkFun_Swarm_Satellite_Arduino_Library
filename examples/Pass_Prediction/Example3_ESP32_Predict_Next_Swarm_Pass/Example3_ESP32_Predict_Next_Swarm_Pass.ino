/*
  Predict the pass of the next Swarm satellite
  By: SparkFun Electronics / Paul Clark
  Date: July 31st, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based extensively on @Hopperpop 's Sgp4Predictor example:
  https://github.com/Hopperpop/Sgp4-Library
  https://github.com/Hopperpop/Sgp4-Library/blob/master/examples/Sgp4Predictor/Sgp4Predictor.ino
  
  This example shows how to predict the pass of the next Swarm satellite.
  
  It uses the CelesTrak Two-Line Element data collected by the previous example: Example2_ESP32_Get_My_Swarm_TLEs
  
  ** If you have enjoyed this code, please consider making a donation to CelesTrak: https://celestrak.org/ **

  This example is written for the SparkFun Thing Plus C but can be adapted for any ESP32 board.

  If the SD card is not detected ("Card Mount Failed"), try adding a 10K pull-up resistor between 19/POCI and 3V3.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus C - ESP32 WROOM

*/

//#define noModemGeospatial // Uncomment this line if you want to define your own location and time for the pass predictions

#ifdef noModemGeospatial

unsigned long unixTime = 1659225600; //                   <-- Update this with the current Unix time if desired

// See https://www.unixtimestamp.com/
// 1659225600 = Sun Jul 31 2022 00:00:00 GMT+0000

double myLat = 55.000; // Latitude 55 degrees North       <-- Update this with your latitude if desired
double myLon = -1.000; // Longitude 1 degree West         <-- Update this with your longitude if desired
double myAlt = 100.00; // Altitude 100m Above Sea Level   <-- Update this with your altitude if desired

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

#include <Sgp4.h> //Click here to get the library: http://librarymanager/All#SparkFun_SGP4_Arduino_Library

Sgp4 sat;

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
  // Swarm Satellite Transceiver MicroMod Function Board PWR_EN
  #ifdef swarmPowerEnablePin
  pinMode(swarmPowerEnablePin, OUTPUT); // Enable modem power 
  digitalWrite(swarmPowerEnablePin, HIGH);
  #endif

  delay(1000);

  Serial.begin(115200);
  Serial.println(F("Example : predict next Swarm pass"));

  while (Serial.available()) Serial.read(); // Empty the serial buffer
  Serial.println(F("Press any key to begin..."));
  while (!Serial.available()); // Wait for a keypress
  
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

  // Call getDateTime to request the most recent Date/Time
  Swarm_M138_DateTimeData_t dateTime; // Allocate memory for the Date/Time
  
  Swarm_M138_Error_e err = mySwarm.getDateTime(&dateTime);

  while (err != SWARM_M138_SUCCESS)
  {
    Serial.print(F("Swarm communication error: "));
    Serial.print((int)err);
    Serial.print(F(" : "));
    Serial.println(mySwarm.modemErrorString(err)); // Convert the error into printable text
    Serial.println(F("The modem may not have acquired a valid GPS date/time reference..."));
    delay(2000);
    err = mySwarm.getDateTime(&dateTime);
  }

  Serial.print(F("getDateTime returned: "));
  Serial.print(dateTime.YYYY);
  Serial.print(F("/"));
  Serial.print(dateTime.MM);
  Serial.print(F("/"));
  Serial.print(dateTime.DD);
  Serial.print(F(" "));
  Serial.print(dateTime.hh);
  Serial.print(F(":"));
  Serial.print(dateTime.mm);
  Serial.print(F(":"));
  Serial.println(dateTime.ss);

  // Call getGeospatialInfo to request the most recent geospatial information
  Swarm_M138_GeospatialData_t info;
  
  err = mySwarm.getGeospatialInfo(&info);
  
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

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Set site latitude[°], longitude[°] and altitude[m]

  sat.site(info.lat, info.lon, info.alt);
#else
  sat.site(myLat, myLon, myAlt);
#endif
  
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
    for (int i = 0; i < 30; i++)
      satelliteName[i] = 0;

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

#ifndef noModemGeospatial
      mySwarm.getDateTime(&dateTime);
      unsigned long unixTime = getUnixEpoch(dateTime);
#endif

      double passStartJD;
      if (Predict(unixTime, &passStartJD)) //Calculates the next overpass
      {
        if ((passStartJD < nextPassJD) && (passStartJD > getJulianFromUnix(unixTime))) // Record the next pass (if not already started)
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
  passinfo overpass; //structure to store overpass info
  sat.initpredpoint( startTime , 0.0 ); //finds the startpoint. startTime is Unix time (not Julian Day)
  
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

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#ifndef noModemGeospatial

// Convert GPS (UTC) time to Unix epoch

const uint32_t SFE_UBLOX_DAYS_FROM_1970_TO_2020 = 18262; // Jan 1st 2020 Epoch = 1577836800 seconds
const uint16_t SFE_UBLOX_DAYS_SINCE_2020[80] =
{
  0, 366, 731, 1096, 1461, 1827, 2192, 2557, 2922, 3288,
  3653, 4018, 4383, 4749, 5114, 5479, 5844, 6210, 6575, 6940,
  7305, 7671, 8036, 8401, 8766, 9132, 9497, 9862, 10227, 10593,
  10958, 11323, 11688, 12054, 12419, 12784, 13149, 13515, 13880, 14245,
  14610, 14976, 15341, 15706, 16071, 16437, 16802, 17167, 17532, 17898,
  18263, 18628, 18993, 19359, 19724, 20089, 20454, 20820, 21185, 21550,
  21915, 22281, 22646, 23011, 23376, 23742, 24107, 24472, 24837, 25203,
  25568, 25933, 26298, 26664, 27029, 27394, 27759, 28125, 28490, 28855
};
const uint16_t SFE_UBLOX_DAYS_SINCE_MONTH[2][12] =
{
  {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}, // Leap Year (Year % 4 == 0)
  {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}  // Normal Year
};

uint32_t getUnixEpoch(Swarm_M138_DateTimeData_t dateTime)
{
  uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020;                                              // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_2020[dateTime.YYYY - 2020];                             // Add on the number of days since 2020
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_MONTH[dateTime.YYYY % 4 == 0 ? 0 : 1][dateTime.MM - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)dateTime.DD - 1;                                                             // Add on the number of days since the 1st of the month
  t *= 24;                                                                                    // Convert to hours
  t += (uint32_t)dateTime.hh;                                                                 // Add on the hour
  t *= 60;                                                                                    // Convert to minutes
  t += (uint32_t)dateTime.mm;                                                                 // Add on the minute
  t *= 60;                                                                                    // Convert to seconds
  t += (uint32_t)dateTime.ss;                                                                 // Add on the second
  return (t);
}

#endif
