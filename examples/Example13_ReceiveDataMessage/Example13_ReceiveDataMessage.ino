/*!
 * @file Example13_ReceiveDataMessage.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Enable message notifications
 *   Set up a callback for the unsolicited $RD receive data messages
 *   Print the messages from the callback
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

// Callback: printDataMsg will be called when a new unsolicited $RD message arrives.
// Note: appID will be NULL for modems with firmware earlier than v1.1.0
void printDataMsg(const uint16_t *appID, const int16_t *rssi, const int16_t *snr, const int16_t *fdev, const char *asciiHex)
{
  Serial.print(F("New $RD data message received:"));
  if (appID != NULL) // appID will be NULL for modems with firmware earlier than v1.1.0
  {
    Serial.print(F("  appID = "));
    Serial.print(*appID);
  }
  Serial.print(F("  RSSI = "));
  Serial.print(*rssi);
  Serial.print(F("  SNR = "));
  Serial.print(*snr);
  Serial.print(F("  FDEV = "));
  Serial.print(*fdev);
  Serial.print(F("  Message (ASCII Hex): "));
  Serial.print(asciiHex);

  // Convert the ASCII Hex into chars and print if printable
  Serial.print(F("  \""));
  for (size_t i = 0; i < strlen(asciiHex); i+= 2)
  {
    uint8_t c = ((asciiHex[i] >= '0') && (asciiHex[i] <= '9')) ? ((asciiHex[i] - '0') << 4) :
                ((asciiHex[i] >= 'a') && (asciiHex[i] <= 'f')) ? ((asciiHex[i] + 10 - 'a') << 4) :
                ((asciiHex[i] >= 'A') && (asciiHex[i] <= 'F')) ? ((asciiHex[i] + 10 - 'A') << 4) : 0;
    c = c | (((asciiHex[i+1] >= '0') && (asciiHex[i+1] <= '9')) ? ((asciiHex[i+1] - '0') << 0) :
                 ((asciiHex[i+1] >= 'a') && (asciiHex[i+1] <= 'f')) ? ((asciiHex[i+1] + 10 - 'a') << 0) :
                 ((asciiHex[i+1] >= 'A') && (asciiHex[i+1] <= 'F')) ? ((asciiHex[i+1] + 10 - 'A') << 0) : 0);
    if (((c >= ' ') && (c <= '~')) || (c == '\r') || (c == '\n'))
      Serial.write(c);
  }
  Serial.println(F("\""));
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

  // Set up the callback for the unsolicited $RD messages. Call printDataMsg when a new message arrives
  mySwarm.setReceiveMessageCallback(&printDataMsg);

  // Enable message notifications
  mySwarm.setMessageNotifications(true);

  // Check if message notifications are enabled
  bool enabled;
  if (mySwarm.getMessageNotifications(&enabled) == SWARM_M138_SUCCESS)
  {
    Serial.print(F("Message notifications are "));
    if (!enabled)
      Serial.print(F("not "));
    Serial.println(F("enabled"));
  }
  else
  {
    Serial.println(F("Swarm communication error"));
  }

  Serial.println(F("Waiting for a new data message..."));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
