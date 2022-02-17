/*!
 * @file Example11_SleepMode.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Set up a callback for the unsolicited $SL sleep mode messages
 *   Print the messages from the callback
 *   Put the modem to sleep using sleepMode
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

// Callback: printSleepMsg will be called when a new unsolicited $SL message arrives.
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_Modem_Status_e.
void printSleepWakeMsg(Swarm_M138_Wake_Cause_e cause)
{
  Serial.print(F("New $SL WAKE message received. Cause is: "));
  Serial.print((int)cause);
  Serial.print(F(" : "));
  if (cause == SWARM_M138_WAKE_CAUSE_GPIO)
    Serial.println(F("GPIO input changed from inactive to active state"));
  else if (cause == SWARM_M138_WAKE_CAUSE_SERIAL)
    Serial.println(F("Activity was detected on the RX pin of the Modem's UART"));
  else if (cause == SWARM_M138_WAKE_CAUSE_TIME)
    Serial.println(F("The S or U parameter time has been reached"));
  else // if (cause == SWARM_M138_WAKE_CAUSE_INVALID)
    Serial.println(F("UNKNOWN / INVALID"));
}

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

  // Set up the callback for the unsolicited $SL messages. Call printSleepWakeMsg when a new message arrives
  mySwarm.setSleepWakeCallback(&printSleepWakeMsg);

  // Set up the callback for the unsolicited $M138 messages. Call printM138Msg when a new message arrives
  mySwarm.setModemStatusCallback(&printM138Msg);

  // Put the modem to sleep for 30 seconds
  Serial.println(F("Putting the modem to sleep for 30 seconds"));
  
  Swarm_M138_Error_e err = mySwarm.sleepMode(30);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.println(F("sleepMode was successful"));
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
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
