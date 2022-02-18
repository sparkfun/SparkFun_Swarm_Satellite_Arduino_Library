/*!
 * @file Example13_ReceiveDataMessage.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
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

  // Set up the callback for the unsolicited $RD messages. Call printDataMsg when a new message arrives
  mySwarm.setReceiveMessageCallback(&printDataMsg);

  Serial.println(F("Waiting for a new data message..."));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
