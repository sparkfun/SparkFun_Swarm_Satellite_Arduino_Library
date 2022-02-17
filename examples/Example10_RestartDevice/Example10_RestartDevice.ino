/*!
 * @file Example10_RestartDevice.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Set up a callback for the unsolicited $M138 modem messages
 *   Print the messages from the callback
 *   Restart the modem using restartDevice
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

unsigned long lastRestartMillis; // Use this to control when the modem is restarted

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printM138Msg will be called when a new unsolicited $M138 message arrives.
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_Modem_Status_e.
void printM138Msg(Swarm_M138_Modem_Status_e status, const char *data)
{
  Serial.print(F("New M138 message received: "));
  Serial.print((int)status);
  Serial.print(F(" : "));
  Serial.print(mySwarm.modemStatusString(status));
  if (*data != 0) // data could be NULL for messages like BOOT_RUNNING
  {
    Serial.print(F(" : "));
    Serial.print(data);
  }
  Serial.println();
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

  // Set up the callback for the unsolicited $M138 messages. Call printM138Msg when a new message arrives
  mySwarm.setModemStatusCallback(&printM138Msg);

  lastRestartMillis = millis(); // Initialize lastRestartMillis
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();

  if (millis() > (lastRestartMillis + 120000)) // Restart the modem every 2 minutes
  {
    Serial.println(F("*** Restarting the modem ***"));
    
    if (mySwarm.restartDevice() == SWARM_M138_SUCCESS)
      Serial.println(F("*** Modem is restarting ***"));
    else
      Serial.println(F("*** Modem restart failed ***"));
    
    lastRestartMillis = millis();
  }

  // Print a countdown to the next restart
  unsigned long secsToRestart = (lastRestartMillis + 120000 - millis()) / 1000;
  static unsigned long countdown;
  if ((secsToRestart % 5 == 0) && (secsToRestart != countdown)) // Print the countdown every 5 seconds
  {
    Serial.print(F("Restarting in "));
    Serial.print(secsToRestart);
    Serial.println(F(" seconds"));
    countdown = secsToRestart;
  }
}
