/*!
 * @file Example5_GeospatialInformation.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Set the rate for the $GN geospatial information message
 *   Set up a callback for the $GN message
 *   Print the unsolicited geospatial information messages from the callback
 * 
 * Want to support open source hardware? Buy a board from SparkFun!
 * SparkX Swarm Serial Breakout : https://www.sparkfun.com/products/19236
 * 
 * @section author Author
 * 
 * This library was written by:
 * Paul Clark
 * SparkFun Electronics
 * February 2022
 * 
 * @section license License
 * 
 * MIT: please see LICENSE.md for the full license information
 * 
 */

#include <SparkFun_Swarm_Satellite_Arduino_Library.h> //Click here to get the library:  http://librarymanager/All#SparkFun_Swarm_Satellite

SWARM_M138 mySwarm;
#define swarmSerial Serial1 // Use Serial1 to communicate with the modem. Change this if required.

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printGeospatial will be called when a new unsolicited $GN geospatial information message arrives
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_GeospatialData_t
//         _____  You can use any name you like for the callback. Use the same name when you call setGeospatialInfoCallback
//        /                         _____  This _must_ be Swarm_M138_GeospatialData_t
//        |                        /                      _____ You can use any name you like for the struct
//        |                        |                     /
//        |                        |                     |
void printGeospatial(const Swarm_M138_GeospatialData_t *info)
{
  Serial.print(F("New geospatial information received:  Lat: "));
  Serial.print(info->lat, 4);
  Serial.print(F("  Lon: "));
  Serial.print(info->lon, 4);
  Serial.print(F("  Alt: "));
  Serial.print(info->alt, 2);
  Serial.print(F("  Course: "));
  Serial.print(info->course, 2);
  Serial.print(F("  Speed: "));
  Serial.println(info->speed, 2);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for the user to open the Serial console
  Serial.println(F("Swarm Satellite example"));
  Serial.println();

  //mySwarm.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (mySwarm.begin(swarmSerial) == false) // Begin communication with the modem
  {
    Serial.println(F("Could not communicate with the modem. Please check the serial connections. Freezing..."));
    while (1)
      ;
  }

  // Just to prove it works, call getGeospatialInfo to request the most recent geospatial information
  Swarm_M138_GeospatialData_t *info = new Swarm_M138_GeospatialData_t; // Allocate memory for the information
  
  mySwarm.getGeospatialInfo(info);
  
  Serial.print(F("getGeospatialInfo returned: "));
  Serial.print(info->lat, 4);
  Serial.print(F(","));
  Serial.print(info->lon, 4);
  Serial.print(F(","));
  Serial.print(info->alt);
  Serial.print(F(","));
  Serial.print(info->course);
  Serial.print(F(","));
  Serial.println(info->speed);
  delete info; // Free the memory


  // Set up the callback for the geospatial information message. Call printGeospatial when a new $GN message arrives
  mySwarm.setGeospatialInfoCallback(&printGeospatial);


  // Set the $GN message rate: send the message every 2 seconds
  Swarm_M138_Error_e err = mySwarm.setGeospatialInfoRate(2);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.println(F("setGeospatialInfoRate was successful"));
  }
  else
  {
    Serial.print(F("Swarm communication error: "));
    Serial.print((int)err);
    Serial.print(F(" : "));
    Serial.print(mySwarm.modemErrorString(err)); // Convert the error into printable text
    if (err == SWARM_M138_ERROR_ERR) // If we received a command error (ERR), print it
    {
      Serial.print(F(" : "));
      Serial.print(mySwarm.commandError); 
      Serial.print(F(" : "));
      Serial.println(mySwarm.commandErrorString((const char *)mySwarm.commandError)); 
    }
    else
      Serial.println();
  }


  // Just to prove it works, call getGeospatialInfoRate to check the message rate
  uint32_t rate;
  
  mySwarm.getGeospatialInfoRate(&rate);

  Serial.print(F("Message rate is "));
  Serial.println(rate);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
