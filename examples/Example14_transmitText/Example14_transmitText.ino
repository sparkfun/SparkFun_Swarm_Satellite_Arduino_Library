/*!
 * @file Example14_transmitText.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Queue a new text message for transmission
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

  // Set up the callback for the unsolicited $TD SENT messages. Call printMessageSent when a new message arrives
  mySwarm.setTransmitDataCallback(&printMessageSent);

  // Send a simple text message
  uint64_t id;
  const char myMessage[] = "Hello world!";
  err = mySwarm.transmitText(myMessage, &id);

  // Alternatives are:
  
  //uint16_t appID = 1234;
  //Swarm_M138_Error_e err = mySwarm.transmitText(myMessage, &id, appID); // Include an appID (0 to 64999)
  
  //uint32_t hold = 3600; // One hour
  //Swarm_M138_Error_e err = mySwarm.transmitTextHold(myMessage, &id, hold); // Include a hold duration
  //Swarm_M138_Error_e err = mySwarm.transmitTextHold(myMessage, &id, hold, appID); // Include a hold duration and an appID
  
  //uint32_t epoch =  1651622400; // May the 4th be with you
  //Swarm_M138_Error_e err = mySwarm.transmitTextExpire(myMessage, &id, epoch); // Include an expire time (epoch)
  //Swarm_M138_Error_e err = mySwarm.transmitTextExpire(myMessage, &id, epoch, appID); // Include an expire time and an appID

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
