/*!
 * @file Example15_transmitBinary.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Queue a new binary message for transmission
 *   Set up a callback for the unsolicited $TD SENT messages
 *   Print the $TD SENT from the callback
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

// Callback: printMessageSent will be called when a new unsolicited $TD SENT message arrives.
void printMessageSent(const int16_t *rssi_sat, const int16_t *snr, const int16_t *fdev, const uint64_t *msg_id)
{
  Serial.print(F("New $TD SENT message received:"));
  Serial.print(F("  RSSI = "));
  Serial.print(*rssi_sat);
  Serial.print(F("  SNR = "));
  Serial.print(*snr);
  Serial.print(F("  FDEV = "));
  Serial.print(*fdev);
  Serial.print(F("  Message ID: "));
  serialPrintUint64_t(*msg_id);
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

  // Set up the callback for the unsolicited $TD SENT messages. Call printMessageSent when a new message arrives
  mySwarm.setTransmitDataCallback(&printMessageSent);

  // Send a simple binary message
  uint64_t id;
  const uint8_t myMessage[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
  size_t len = 8;
  Swarm_M138_Error_e err = mySwarm.transmitBinary(myMessage, len, &id);

  // Alternatives are:
  
  //uint16_t appID = 1234;
  //Swarm_M138_Error_e err = mySwarm.transmitBinary(myMessage, len, &id, appID); // Include an appID (0 to 64999)
  
  //uint32_t hold = 3600; // One hour
  //Swarm_M138_Error_e err = mySwarm.transmitBinaryHold(myMessage, len, &id, hold); // Include a hold duration
  //Swarm_M138_Error_e err = mySwarm.transmitBinaryHold(myMessage, len, &id, hold, appID); // Include a hold duration and an appID
  
  //uint32_t epoch =  1651622400; // May the 4th be with you
  //Swarm_M138_Error_e err = mySwarm.transmitBinaryExpire(myMessage, len, &id, epoch); // Include an expire time (epoch)
  //Swarm_M138_Error_e err = mySwarm.transmitBinaryExpire(myMessage, len, &id, epoch, appID); // Include an expire time and an appID

  // Check if the message was queued successfully
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.print(F("The message has been added to the transmit queue. The message ID is "));
    serialPrintUint64_t(id);
    Serial.println();
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

  Serial.println(F("Waiting for the $TD SENT"));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void serialPrintUint64_t(uint64_t theNum)
{
  // Convert uint64_t to string
  // Based on printLLNumber by robtillaart
  // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
  
  char rev[21]; // Char array to hold to theNum (reversed order)
  char fwd[21]; // Char array to hold to theNum (correct order)
  unsigned int i = 0;
  if (theNum == 0ULL) // if theNum is zero, set fwd to "0"
  {
    fwd[0] = '0';
    fwd[1] = 0; // mark the end with a NULL
  }
  else
  {
    while (theNum > 0)
    {
      rev[i++] = (theNum % 10) + '0'; // divide by 10, convert the remainder to char
      theNum /= 10; // divide by 10
    }
    unsigned int j = 0;
    while (i > 0)
    {
      fwd[j++] = rev[--i]; // reverse the order
      fwd[j] = 0; // mark the end with a NULL
    }
  }

  Serial.print(fwd);
}
