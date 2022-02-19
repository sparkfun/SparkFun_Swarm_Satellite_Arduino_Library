/*!
 * @file Example17_RxMessageManagement.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Get the received message count
 *   Delete all read messages
 *   Delete all received messages
 *   Mark all unread messages as read
 *   Read the newest unread message
 *   Read the oldest unread message
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
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  while (Serial.available()) // Empty the serial buffer
    Serial.read();
  
  // Print a menu of the messages in the transmit queue
  Serial.println();
  Serial.println(F("Swarm RX Message Management"));
  Serial.println();
  
  uint16_t totalCount;
  mySwarm.getRxMessageCount(&totalCount); // Get the total number of received messages (read and unread)

  Serial.print(F("There are "));
  Serial.print(totalCount);
  Serial.print(F(" received messages of which "));

  uint16_t unreadCount;
  mySwarm.getRxMessageCount(&unreadCount, true); // Get the number of unread messages
  
  Serial.print(unreadCount);
  Serial.println(F(" are unread"));
  Serial.println();

  if (totalCount > 0)
  {
    Serial.println(F("Enter:"));
    Serial.println(F("d to delete all read messages"));
    Serial.println(F("D to delete all messages (read and unread)"));
    Serial.println(F("M to mark all messages as read"));

    if (unreadCount > 0)
    {
      Serial.println(F("N to read the newest unread message"));
      Serial.println(F("O to read the oldest unread message"));      
    }
    
    while (!Serial.available()) // Wait for the user to enter a character
      ;
  
    char c = Serial.read(); // Read the serial character

    Swarm_M138_Error_e err;
  
    if (c == 'd') // Delete read messages
    {
      err = mySwarm.deleteAllRxMessages(true);
    }
    
    else if (c == 'D') // Delete all messages
    {
      err = mySwarm.deleteAllRxMessages(false);
    }
  
    else if (c == 'M') // Mark all messages as read
    {
      err = mySwarm.markAllRxMessages();
    }
  
    else if ((c == 'N') || (c == 'O')) // Read a message
    {
      // Allocate storage for the message. The message could be up to 2 * 192 bytes long. Include space for a null on the end.
      char *asciiHex = new char[385];
      uint32_t epoch;
      uint16_t appID;
      uint64_t msg_id;

      if (c == 'N')
        err = mySwarm.readNewestMessage(asciiHex, 385, &msg_id, &epoch, &appID); // Read the message
      else
        err = mySwarm.readOldestMessage(asciiHex, 385, &msg_id, &epoch, &appID); // Read the message

      if (err == SWARM_M138_SUCCESS) // If the read was successful, print the message
      {
        Serial.println();
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
      
      delete[] asciiHex; // Delete the char storage
    }

    if ((c == 'd') || (c == 'D') || (c == 'M') || (c == 'N') || (c == 'O'))
    {
      // Print the result
      if (err == SWARM_M138_SUCCESS)
      {
        Serial.println(F("The command was successful"));
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
    }
  }
  else
  {
    while (!Serial.available()) // Wait for the user to enter something
      ;    
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
