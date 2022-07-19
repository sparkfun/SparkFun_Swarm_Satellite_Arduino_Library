/*!
 * @file Example19_ReadSingleRxMessage.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Read a single message using its msg_id
 *   The message ID is provided when you read the message using readOldestMessage or readNewestMessage
 *   Note: There doesn't appear to be a way to list all messages and retrieve their IDs (like you can with $MT L=U)
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

  // Allocate storage for the message. The message could be up to 2 * 192 bytes long. Include space for a null on the end.
  char *asciiHex = new char[385];
  uint32_t epoch;
  uint16_t appID;
  
  uint64_t msg_id = 21990234193962; // <== Enter the message ID here

  Swarm_M138_Error_e err = mySwarm.readMessage(msg_id, asciiHex, 385, &epoch, &appID); // Read the message

  if (err == SWARM_M138_SUCCESS) // If the read was successful, print the message
  {
    Serial.print(F("Message contents in ASCII Hex: "));
    Serial.println(asciiHex); // Print the message contents in ASCII Hex

    // Convert the ASCII Hex into chars and print if printable
    Serial.print(F("Message contents (if printable): "));
    for (size_t i = 0; i < strlen(asciiHex); i+= 2)
    {
      uint8_t cc = ((asciiHex[i] >= '0') && (asciiHex[i] <= '9')) ? ((asciiHex[i] - '0') << 4) :
                   ((asciiHex[i] >= 'a') && (asciiHex[i] <= 'f')) ? ((asciiHex[i] + 10 - 'a') << 4) :
                   ((asciiHex[i] >= 'A') && (asciiHex[i] <= 'F')) ? ((asciiHex[i] + 10 - 'A') << 4) : 0;
      cc = cc | (((asciiHex[i+1] >= '0') && (asciiHex[i+1] <= '9')) ? ((asciiHex[i+1] - '0') << 0) :
                  ((asciiHex[i+1] >= 'a') && (asciiHex[i+1] <= 'f')) ? ((asciiHex[i+1] + 10 - 'a') << 0) :
                  ((asciiHex[i+1] >= 'A') && (asciiHex[i+1] <= 'F')) ? ((asciiHex[i+1] + 10 - 'A') << 0) : 0);
      if (((cc >= ' ') && (cc <= '~')) || (cc == '\r') || (cc == '\n')) // Check if cc is printable
        Serial.write(cc);
    }
    Serial.println();

    Serial.print(F("Message ID: "));
    serialPrintUint64_t(msg_id);
    Serial.print(F("  Message epoch: "));
    Serial.print(epoch);
    Serial.print(F("  Message appID: "));
    Serial.println(appID);
    Serial.println();
  }
  else
  {
    Serial.print(F("Swarm error: "));
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
  
  delete[] asciiHex; // Delete the char storage
  
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  // Nothing to do here
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
