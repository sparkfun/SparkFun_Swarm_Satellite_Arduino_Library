/*!
 * @file Example7_GPSFixQuality.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Set the rate for the $GS GPS fix quality message
 *   Set up a callback for the $GS message
 *   Print the unsolicited fix quality messages from the callback
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

// Callback: printGPSFix will be called when a new unsolicited $GS GPS fix quality message arrives
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_GPS_Fix_Quality_t
//         _____  You can use any name you like for the callback. Use the same name when you call setGpsFixQualityCallback
//        /                         _____  This _must_ be Swarm_M138_GPS_Fix_Quality_t
//        |                        /                      _____ You can use any name you like for the struct
//        |                        |                     /
//        |                        |                     |
void printGPSFix(const Swarm_M138_GPS_Fix_Quality_t *fixQuality)
{
  Serial.print(F("New GPS fix quality received:  hdop: "));
  Serial.print(fixQuality->hdop);
  Serial.print(F("  vdop: "));
  Serial.print(fixQuality->vdop);
  Serial.print(F("  gnss_sats: "));
  Serial.print(fixQuality->gnss_sats);
  Serial.print(F("  fix_type: "));
  Serial.print(fixQuality->fix_type);
  Serial.print(F(" ("));
  if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_NF)
    Serial.print(F("No fix"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_DR)
    Serial.print(F("Dead reckoning only solution"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_G2)
    Serial.print(F("Standalone 2D solution"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_G3)
    Serial.print(F("Standalone 3D solution"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_D2)
    Serial.print(F("Differential 2D solution"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_D3)
    Serial.print(F("Differential 3D solution"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_RK)
    Serial.print(F("Combined GNSS + dead reckoning solution"));
  else if (fixQuality->fix_type == SWARM_M138_GPS_FIX_TYPE_TT)
    Serial.print(F("Time only solution"));
  else
    Serial.print(F("INVALID"));
  Serial.println(F(")"));
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

  // Just to prove it works, call getGpsFixQuality to request the most recent GPS fix qwuality
  Swarm_M138_GPS_Fix_Quality_t *fixQuality = new Swarm_M138_GPS_Fix_Quality_t; // Allocate memory for the information
  
  mySwarm.getGpsFixQuality(fixQuality);
  
  Serial.print(F("getGpsFixQuality returned: "));
  Serial.print(fixQuality->hdop);
  Serial.print(F(","));
  Serial.print(fixQuality->vdop);
  Serial.print(F(","));
  Serial.print(fixQuality->gnss_sats);
  Serial.print(F(","));
  Serial.print(fixQuality->unused);
  Serial.print(F(","));
  Serial.println(fixQuality->fix_type);

  delete fixQuality; // Free the memory


  // Set up the callback for the GPS fix quality message. Call printGPSFix when a new $GS message arrives
  mySwarm.setGpsFixQualityCallback(&printGPSFix);


  // Set the $GS message rate: send the message every 2 seconds
  Swarm_M138_Error_e err = mySwarm.setGpsFixQualityRate(2);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.println(F("setGpsFixQualityRate was successful"));
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


  // Just to prove it works, call getGpsFixQualityRate to check the message rate
  uint32_t rate;
  
  mySwarm.getGpsFixQualityRate(&rate);
  
  Serial.print(F("Message rate is "));
  Serial.println(rate);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  mySwarm.checkUnsolicitedMsg();
}
