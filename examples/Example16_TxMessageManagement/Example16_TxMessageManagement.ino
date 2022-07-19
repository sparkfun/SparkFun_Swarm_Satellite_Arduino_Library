/*!
 * @file Example16_TxMessageManagement.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Queue a new text message for transmission
 *   List the IDs of all unsent messages
 *   List the contents of a single unsent message
 *   Delete a single unsent message
 *   Delete all unsent messages
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

  // Get the number of untransmitted messages
  uint16_t count;
  mySwarm.getUnsentMessageCount(&count);
  Serial.print(F("There are "));
  Serial.print(count);
  Serial.println(F(" unsent messages in the TX queue"));
  Serial.println();

  if (count == 0) // If the queue is empty, add four new messages for transmission
  {
    Serial.println(F("Adding four new ones:"));

    // Queue four messages
    uint64_t id;
    const char one[] = "One, ";
    const char two[] = "Two, ";
    const char three[] = "Three, ";
    const char four[] = "Four!";
    if (mySwarm.transmitText(one, &id) == SWARM_M138_SUCCESS)
    {
      Serial.print(F("Message one has been added to the transmit queue. The message ID is "));
      serialPrintUint64_t(id);
      Serial.println();
    }
    if (mySwarm.transmitText(two, &id) == SWARM_M138_SUCCESS)
    {
      Serial.print(F("Message two has been added to the transmit queue. The message ID is "));
      serialPrintUint64_t(id);
      Serial.println();
    }
    if (mySwarm.transmitText(three, &id) == SWARM_M138_SUCCESS)
    {
      Serial.print(F("Message three has been added to the transmit queue. The message ID is "));
      serialPrintUint64_t(id);
      Serial.println();
    }
    if (mySwarm.transmitText(four, &id) == SWARM_M138_SUCCESS)
    {
      Serial.print(F("Message four has been added to the transmit queue. The message ID is "));
      serialPrintUint64_t(id);
      Serial.println();
    }
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  while (Serial.available()) // Empty the serial buffer
    Serial.read();
  
  // Print a menu of the messages in the transmit queue
  Serial.println();
  Serial.println(F("Swarm TX Message Management"));
  Serial.println();
  
  uint16_t count;
  mySwarm.getUnsentMessageCount(&count); // Get the number of untransmitted messages

  Serial.print(F("There are "));
  Serial.print(count);
  Serial.println(F(" unsent messages in the TX queue"));
  Serial.println();

  if (count > 0)
  {
    Serial.println(F("Enter:"));
    Serial.println(F("A to delete all unsent messages"));
    
    while (!Serial.available()) // Wait for the user to enter a character
      mySwarm.checkUnsolicitedMsg(); // Keep emptying the backlog
  
    char c = Serial.read(); // Read the serial character
  
    if (c == 'A') // Delete all messages
    {
      mySwarm.deleteAllTxMessages();
    }
  }
  else
  {
    while (!Serial.available()) // Wait for the user to enter something
      mySwarm.checkUnsolicitedMsg(); // Keep emptying the backlog
  }
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
