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

  // Wait until the modem has valid Date/Time
  Swarm_M138_DateTimeData_t dateTime;
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

  // Set up the callback for the unsolicited $SL messages. Call printSleepWakeMsg when a new message arrives
  mySwarm.setSleepWakeCallback(&printSleepWakeMsg);

  // Set up the callback for the unsolicited $M138 messages. Call printM138Msg when a new message arrives
  mySwarm.setModemStatusCallback(&printM138Msg);

  // Put the modem to sleep for 30 seconds
  Serial.println(F("Putting the modem to sleep for 30 seconds"));
  
  err = mySwarm.sleepMode(30);
  
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
