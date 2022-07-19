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
  // Swarm Satellite Transceiver MicroMod Function Board PWR_EN
  #ifdef swarmPowerEnablePin
  pinMode(swarmPowerEnablePin, OUTPUT); // Enable modem power 
  digitalWrite(swarmPowerEnablePin, HIGH);
  #endif

  delay(1000);
  
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for the user to open the Serial console
  Serial.println(F("Swarm Satellite example"));
  Serial.println();

  //mySwarm.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  bool modemBegun = mySwarm.begin(swarmSerial); // Begin communication with the modem
  
  while (!modemBegun) // If the begin failed, keep trying to begin communication with the modem
  {
    Serial.println(F("Could not communicate with the modem. It may still be booting..."));
    delay(2000);
    modemBegun = mySwarm.begin(swarmSerial);
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
