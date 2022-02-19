/*!
 * @file Example4_GPSJammingSpoofing.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Set the rate for the $GJ GPS Jamming/Spoofing Indication message
 *   Set up a callback for the $GJ message
 *   Print the unsolicited messages from the callback
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

// Callback: printGPSJamming will be called when a new unsolicited $GJ message arrives
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_GPS_Jamming_Indication_t
//         _____  You can use any name you like for the callback. Use the same name when you call setGpsJammingCallback
//        /                           _____  This _must_ be Swarm_M138_GPS_Jamming_Indication_t
//        |                          /                               _____ You can use any name you like for the struct
//        |                          |                              /
//        |                          |                              |
void printGPSJamming(const Swarm_M138_GPS_Jamming_Indication_t *jamming)
{
  Serial.print(F("New GPS jamming/spoofing indication received:  spoof_state : "));
  
  Serial.print(jamming->spoof_state);
  Serial.print(F(" ("));
  if (jamming->spoof_state == 0)
    Serial.print(F("Spoofing unknown or deactivated"));
  else if (jamming->spoof_state == 1)
    Serial.print(F("No spoofing indicated"));
  else if (jamming->spoof_state == 2)
    Serial.print(F("Spoofing indicated"));
  else // if (jamming->spoof_state == 3)
    Serial.print(F("Multiple spoofing indications"));

  Serial.print(F(")  jamming_level : "));
  Serial.println(jamming->jamming_level);
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

  // Just to prove it works, call getGpsJammingIndication to request the most recent jamming indication
  Swarm_M138_GPS_Jamming_Indication_t *jamming = new Swarm_M138_GPS_Jamming_Indication_t; // Allocate memory for the jamming indication
  
  mySwarm.getGpsJammingIndication(jamming);
  
  Serial.print(F("getGPSJammingIndication returned: "));
  Serial.print(jamming->spoof_state);
  Serial.print(F(","));
  Serial.println(jamming->jamming_level);
  
  delete jamming; // Free the memory


  // Set up the callback for the jamming indication message. Call printGPSJamming when a new $GJ message arrives
  mySwarm.setGpsJammingCallback(&printGPSJamming);


  // Set the jamming message rate: send the message every 2 seconds
  Swarm_M138_Error_e err = mySwarm.setGpsJammingIndicationRate(2);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.println(F("setGpsJammingIndicationRate was successful"));
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


  // Just to prove it works, call getGpsJammingIndicationRate to check the message rate
  uint32_t rate;
  
  mySwarm.getGpsJammingIndicationRate(&rate);

  Serial.print(F("Message rate is "));
  Serial.println(rate);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
