/*!
 * @file Example9_ReceiveTest.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Set the rate for the $RT receive test message
 *   Set up a callback for the $RT message
 *   Print the unsolicited receive test messages from the callback
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

// Callback: printRxTest will be called when a new unsolicited $RT receive test message arrives
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_Receive_Test_t
//         _____  You can use any name you like for the callback. Use the same name when you call setReceiveTestCallback
//        /                         _____  This _must_ be Swarm_M138_Receive_Test_t
//        |                        /                  _____ You can use any name you like for the struct
//        |                        |                 /
//        |                        |                 |
void printRxTest(const Swarm_M138_Receive_Test_t *rxTest)
{
  Serial.print(F("New receive test message received:"));
  if (rxTest->background) // Check if rxTest contains only the background RSSI
  {
    Serial.print(F("  rssi_background: "));
    Serial.print(rxTest->rssi_background);
    if (rxTest->rssi_background <= -105)
      Serial.println(F(" (Great)"));
    else if (rxTest->rssi_background <= -100)
      Serial.println(F(" (Good)"));
    else if (rxTest->rssi_background <= -97)
      Serial.println(F(" (OK)"));
    else if (rxTest->rssi_background <= -93)
      Serial.println(F(" (Marginal)"));
    else
      Serial.println(F(" (Bad)"));
  }
  else
  {
    Serial.print(F("  rssi_sat: "));
    Serial.print(rxTest->rssi_sat);
    Serial.print(F("  snr: "));
    Serial.print(rxTest->snr);
    Serial.print(F("  fdev: "));
    Serial.print(rxTest->fdev);
    Serial.print(F("  "));
    Serial.print(rxTest->time.YYYY);
    Serial.print(F("/"));
    if (rxTest->time.MM < 10) Serial.print(F("0")); Serial.print(rxTest->time.MM); // Print the month. Add a leading zero if required
    Serial.print(F("/"));
    if (rxTest->time.DD < 10) Serial.print(F("0")); Serial.print(rxTest->time.DD); // Print the day of month. Add a leading zero if required
    Serial.print(F(" "));
    if (rxTest->time.hh < 10) Serial.print(F("0")); Serial.print(rxTest->time.hh); // Print the hour. Add a leading zero if required
    Serial.print(F(":"));
    if (rxTest->time.mm < 10) Serial.print(F("0")); Serial.print(rxTest->time.mm); // Print the minute. Add a leading zero if required
    Serial.print(F(":"));
    if (rxTest->time.ss < 10) Serial.print(F("0")); Serial.print(rxTest->time.ss); // Print the second. Add a leading zero if required
    Serial.print(F("  sat_id: 0x"));
    Serial.println(rxTest->sat_id, HEX);
  }
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

  // Just to prove it works, call getReceiveTest to request the most recent RSSI etc.
  Swarm_M138_Receive_Test_t *rxTest = new Swarm_M138_Receive_Test_t; // Allocate memory for the information
  
  mySwarm.getReceiveTest(rxTest);
  
  Serial.print(F("getReceiveTest returned:"));
  if (rxTest->background) // Check if rxTest contains only the background RSSI
  {
    Serial.print(F("  rssi_background: "));
    Serial.println(rxTest->rssi_background);
  }
  else
  {
    Serial.print(F("  rssi_sat: "));
    Serial.print(rxTest->rssi_sat);
    Serial.print(F("  snr: "));
    Serial.print(rxTest->snr);
    Serial.print(F("  fdev: "));
    Serial.print(rxTest->fdev);
    Serial.print(F("  "));
    Serial.print(rxTest->time.YYYY);
    Serial.print(F("/"));
    if (rxTest->time.MM < 10) Serial.print(F("0")); Serial.print(rxTest->time.MM); // Print the month. Add a leading zero if required
    Serial.print(F("/"));
    if (rxTest->time.DD < 10) Serial.print(F("0")); Serial.print(rxTest->time.DD); // Print the day of month. Add a leading zero if required
    Serial.print(F(" "));
    if (rxTest->time.hh < 10) Serial.print(F("0")); Serial.print(rxTest->time.hh); // Print the hour. Add a leading zero if required
    Serial.print(F(":"));
    if (rxTest->time.mm < 10) Serial.print(F("0")); Serial.print(rxTest->time.mm); // Print the minute. Add a leading zero if required
    Serial.print(F(":"));
    if (rxTest->time.ss < 10) Serial.print(F("0")); Serial.print(rxTest->time.ss); // Print the second. Add a leading zero if required
    Serial.print(F("  sat_id: 0x"));
    Serial.println(rxTest->sat_id, HEX);
  }

  delete rxTest; // Free the memory


  // Set up the callback for the receive test message. Call printRxTest when a new $RT message arrives
  mySwarm.setReceiveTestCallback(&printRxTest);


  // Set the $RT message rate: send the message every 2 seconds
  Swarm_M138_Error_e err = mySwarm.setReceiveTestRate(2);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.println(F("setReceiveTestRate was successful"));
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


  // Just to prove it works, call getReceiveTestRate to check the message rate
  uint32_t rate;
  
  mySwarm.getReceiveTestRate(&rate);

  Serial.print(F("Message rate is "));
  Serial.println(rate);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
