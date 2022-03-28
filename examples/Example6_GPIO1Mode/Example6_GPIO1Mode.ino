/*!
 * @file Example6_GPIO1Mode.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to set and get the GPIO1 pin mode.
 * 
 * Want to support open source hardware? Buy a board from SparkFun!
 * SparkX Swarm Serial Breakout : https://www.sparkfun.com/products/19236
 * 
 * @section author Author
 * 
 * This library was written by:
 * Paul Clark
 * SparkFun Electronics
 * January 2022
 * 
 * @section license License
 * 
 * MIT: please see LICENSE.md for the full license information
 * 
 */

#include <SparkFun_Swarm_Satellite_Arduino_Library.h> //Click here to get the library:  http://librarymanager/All#SparkFun_Swarm_Satellite

SWARM_M138 mySwarm;
#define swarmSerial Serial1 // Use Serial1 to communicate with the modem. Change this if required.

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

  // Set the GPIO1 pin mode
  // Possible choices are:
  //  SWARM_M138_GPIO1_ANALOG
  //  SWARM_M138_GPIO1_ADC
  //  SWARM_M138_GPIO1_INPUT
  //  SWARM_M138_GPIO1_EXIT_SLEEP_LOW_HIGH
  //  SWARM_M138_GPIO1_EXIT_SLEEP_HIGH_LOW
  //  SWARM_M138_GPIO1_OUTPUT_LOW
  //  SWARM_M138_GPIO1_OUTPUT_HIGH
  //  SWARM_M138_GPIO1_MESSAGES_UNREAD_LOW
  //  SWARM_M138_GPIO1_MESSAGES_UNREAD_HIGH
  //  SWARM_M138_GPIO1_MESSAGES_UNSENT_LOW
  //  SWARM_M138_GPIO1_MESSAGES_UNSENT_HIGH
  //  SWARM_M138_GPIO1_MESSAGES_UNREAD_UNSENT_LOW
  //  SWARM_M138_GPIO1_MESSAGES_UNREAD_UNSENT_HIGH
  //  SWARM_M138_GPIO1_SLEEP_MODE_LOW
  //  SWARM_M138_GPIO1_SLEEP_MODE_HIGH
  Swarm_M138_Error_e err = mySwarm.setGPIO1Mode(SWARM_M138_GPIO1_ADC);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.println(F("setGPIO1Mode was successful"));
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


  // Just to prove it worked, call getGPIO1Mode to check the pin mode
  Swarm_M138_GPIO1_Mode_e mode;
  
  mySwarm.getGPIO1Mode(&mode);
  
  Serial.print(F("The GPIO1 mode is: "));
  Serial.print((int)mode);
  Serial.print(F(" : "));
  switch (mode)
  {
    case SWARM_M138_GPIO1_ANALOG:
      Serial.println(F(" Analog, pin is internally disconnected and not used (default)"));
      break;
    case SWARM_M138_GPIO1_ADC:
      Serial.println(F(" Analog ADC, pin can be read to measure input voltage (0-3.3V)"));
      break;
    case SWARM_M138_GPIO1_INPUT:
      Serial.println(F(" Input, pin can be read as a general purpose digital input (High or Low)"));
      break;
    case SWARM_M138_GPIO1_EXIT_SLEEP_LOW_HIGH:
      Serial.println(F(" Input, low-to-high transition exits sleep mode"));
      break;
    case SWARM_M138_GPIO1_EXIT_SLEEP_HIGH_LOW:
      Serial.println(F(" Input, high-to-low transition exits sleep mode"));
      break;
    case SWARM_M138_GPIO1_OUTPUT_LOW:
      Serial.println(F(" Output (open drain), set low"));
      break;
    case SWARM_M138_GPIO1_OUTPUT_HIGH:
      Serial.println(F(" Output (open drain), set high/open"));
      break;
    case SWARM_M138_GPIO1_MESSAGES_UNREAD_LOW:
      Serial.println(F(" Output (open drain), low indicates unread messages pending for user"));
      break;
    case SWARM_M138_GPIO1_MESSAGES_UNREAD_HIGH:
      Serial.println(F(" Output (open drain), high/open indicates unread messages pending for user"));
      break;
    case SWARM_M138_GPIO1_MESSAGES_UNSENT_LOW:
      Serial.println(F(" Output (open drain), low indicates unsent messages pending for transmit"));
      break;
    case SWARM_M138_GPIO1_MESSAGES_UNSENT_HIGH:
      Serial.println(F(" Output (open drain), high/open indicates unsent messages pending for transmit"));
      break;
    case SWARM_M138_GPIO1_MESSAGES_UNREAD_UNSENT_LOW:
      Serial.println(F(" Output (open drain), low indicates unread or unsent messages"));
      break;
    case SWARM_M138_GPIO1_MESSAGES_UNREAD_UNSENT_HIGH:
      Serial.println(F(" Output (open drain), high/open indicates unread or unsent messages"));
      break;
    case SWARM_M138_GPIO1_SLEEP_MODE_LOW:
      Serial.println(F(" Output (open drain), low indicates sleep mode is active. Otherwise output is high/open"));
      break;
    case SWARM_M138_GPIO1_SLEEP_MODE_HIGH:
      Serial.println(F(" Output (open drain), high/open indicates sleep mode is active. Otherwise output is low"));
      break;
    default:
      Serial.println(F(" UNKNOWN"));
      break;      
  }

  // Just to prove we can, call getGPIO1voltage to check the pin voltage (only valid for modes 1 and 2)
  float voltage;
  err = mySwarm.readGPIO1voltage(&voltage);
  
  if (err == SWARM_M138_SUCCESS)
  {
    Serial.print(F("GPIO1 voltage is: "));
    Serial.println(voltage, 3);
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

void loop()
{
  //Nothing to do here
}
