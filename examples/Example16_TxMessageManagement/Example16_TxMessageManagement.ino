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
  Serial.println(F(" messages in the TX queue"));
  Serial.println();

  if (count > 0)
  {
    if (count > 10)
    {
      count = 10; // Limit the menu to the first ten messages
      Serial.println(F("The first 10 messages are:"));
    }
    else
      Serial.println(F("The messages are:"));

    uint64_t *ids = new uint64_t[count]; // Create an array of uint64_t to store the message IDs
    mySwarm.listTxMessagesIDs(ids, count); // Ask the modem for a list of the IDs
  
    for (uint16_t i = 0; i < count; i++) // Print all the message IDs
    {
      Serial.print(i);
      Serial.print(F(" : "));
      serialPrintUint64_t(ids[i]);
      Serial.println();
    }
  
    Serial.println();
    Serial.println(F("Enter:"));
    Serial.println(F("L followed by the message number to list a message. E.g. L1"));
    Serial.println(F("D followed by the message number to delete a message. E.g. D0"));
    Serial.println(F("A to delete all messages"));
    
    while (!Serial.available()) // Wait for the user to enter a character
      ;
  
    char c = Serial.read(); // Read the serial character
  
    if (c == 'L') // List a message
    {
      while (!Serial.available()) // Wait for the user to enter the message number
        ;
      c = Serial.read(); // Read the serial character
      if ((c >= '0') && (c < ('0' + count)) && (c <= '9')) // Check for a valid number
      {
        // Allocate storage for the message. The message could be up to 2 * 192 bytes long. Include space for a null on the end.
        char *asciiHex = new char[385];
        uint32_t epoch;
        uint16_t appID;
        mySwarm.listTxMessage(ids[c - '0'], asciiHex, 385, &epoch, &appID); // List the message
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

        Serial.print(F("Message epoch: "));
        Serial.print(epoch);
        Serial.print(F("  Message appID: "));
        Serial.println(appID);
        Serial.println();
        
        delete[] asciiHex; // Delete the char storage
      }
    }
    
    else if (c == 'D') // Delete a message
    {
      while (!Serial.available()) // Wait for the user to enter the message number
        ;
      c = Serial.read(); // Read the serial character
      if ((c >= '0') && (c < ('0' + count)) && (c <= '9')) // Check for a valid number
      {
        mySwarm.deleteTxMessage(ids[c - '0']); // Delete the message
      }
    }
    
    else if (c == 'A') // Delete all messages
    {
      mySwarm.deleteAllTxMessages();
    }
  
    delete[] ids; // Delete the ID storage
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
