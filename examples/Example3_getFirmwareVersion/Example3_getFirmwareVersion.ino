/*!
 * @file Example3_getFirmwareVersion.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to read the firmware version) from the modem.
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

  char *firmwareVersion = new char[SWARM_M138_MEM_ALLOC_FV]; // Create storage for the configuration settings

  Swarm_M138_Error_e err = mySwarm.getFirmwareVersion(firmwareVersion);
  
  if (err == SWARM_M138_SUCCESS) // Get the settings
  {
    Serial.print(F("The firmware version is: "));
    Serial.println(firmwareVersion);
  }
  else
  {
    Serial.print(F("Swarm communication error: "));
    Serial.print((int)err);
    Serial.print(F(" : "));
    Serial.println(mySwarm.modemErrorString(err)); // Convert the error into printable text
  }

  delete[] firmwareVersion; // Free the storage
}

void loop()
{
  //Nothing to do here
}
