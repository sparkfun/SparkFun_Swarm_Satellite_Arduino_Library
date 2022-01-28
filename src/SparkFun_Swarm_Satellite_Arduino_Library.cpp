/*!
 * @file SparkFun_Swarm_Satellite_Arduino_Library.cpp
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Introduction
 * 
 * This library facilitates communication with the Swarm M138 satellite modem.
 * 
 * Want to support open source hardware? Buy a board from SparkFun!
 * <br>SparkX Swarm Serial Breakout (SPX-19236): https://www.sparkfun.com/products/19236
 *  * 
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

#include "SparkFun_Swarm_Satellite_Arduino_Library.h"

SWARM_M138::SWARM_M138(int gpio1Pin)
{
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  _softSerial = NULL;
#endif
  _hardSerial = NULL;
  _baud = SWARM_M138_SERIAL_BAUD_RATE;
  _i2cPort = NULL;
  _address = SFE_QWIIC_SWARM_DEFAULT_I2C_ADDRESS;
  _debugPort = NULL;
  _printDebug = false;
  _gpio1Pin = gpio1Pin;
  _checkUnsolicitedMsgReentrant = false;
  _swarmDateTimeCallback = NULL;
  _lastI2cCheck = millis();
}

#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
/**************************************************************************/
/*!
    @brief  Begin communication with the Swarm M138 modem
    @param  softSerial
            The software serial port to be used to communicate with the modem
    @return True if communication with the modem was successful, otherwise false
*/
/**************************************************************************/
bool SWARM_M138::begin(SoftwareSerial &softSerial)
{
  if (!initializeBuffers())
    return false;
    
  _softSerial = &softSerial;

  // There's no 'easy' way to tell if the serial port has already been begun for us.
  // We have to assume it has not been begun and so do it here.
  // For special cases like Software Serial on ESP32, we need to begin _and_ end the port externally
  // _before_ calling the SWARM_M138 .begin.
  beginSerial(_baud);

  return (isConnected());
}
#endif

/**************************************************************************/
/*!
    @brief  Begin communication with the Swarm M138 modem
    @param  hardSerial
            The hardware serial port to be used to communicate with the modem
    @return True if communication with the modem was successful, otherwise false
*/
/**************************************************************************/
bool SWARM_M138::begin(HardwareSerial &hardSerial)
{
  if (!initializeBuffers())
    return false;
    
  _hardSerial = &hardSerial;

  // There's no 'easy' way to tell if the serial port has already been begun for us.
  // We have to assume it has not been begun and so do it here.
  // For special cases like Software Serial on ESP32, we need to begin _and_ end the port externally
  // _before_ calling the SWARM_M138 .begin.
  beginSerial(_baud);

  return (isConnected());
}

/**************************************************************************/
/*!
    @brief  Begin communication with the Swarm M138 modem
    @param  deviceAddress
            The I2C address of the Qwiic Swarm.
            Default is SFE_QWIIC_SWARM_DEFAULT_I2C_ADDRESS 0x52.
    @param  wirePort
            The TwoWire (I2C) port used to communicate with the Power Board.
            Default is Wire.
    @return True if communication with the modem was successful, otherwise false
*/
/**************************************************************************/
bool SWARM_M138::begin(byte deviceAddress, TwoWire &wirePort)
{
  if (!initializeBuffers())
    return false;

  _i2cPort = &wirePort;
  _address = deviceAddress;

  return (isConnected());
}

/**************************************************************************/
/*!
    @brief  Check if the modem is connected and responding by getting
            the configuration settings
    @return True if successful
            False if unsuccessful
*/
/**************************************************************************/
bool SWARM_M138::isConnected(void)
{
  uint32_t dev_ID = 0;
  return (getDeviceID(&dev_ID) == SWARM_M138_ERROR_SUCCESS);
}

// Private: allocate memory for the serial buffers and clear it
bool SWARM_M138::initializeBuffers()
{
  _swarmRxBuffer = new char[_RxBuffSize];
  if (_swarmRxBuffer == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("begin: not enough memory for _swarmRxBuffer!"));
    return false;
  }
  memset(_swarmRxBuffer, 0, _RxBuffSize);

  _pruneBuffer = new char[_RxBuffSize];
  if (_pruneBuffer == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("begin: not enough memory for _pruneBuffer!"));
    return false;
  }
  memset(_pruneBuffer, 0, _RxBuffSize);

  _swarmBacklog = new char[_RxBuffSize];
  if (_swarmBacklog == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("begin: not enough memory for _swarmBacklog!"));
    return false;
  }
  memset(_swarmBacklog, 0, _RxBuffSize);

  return true;
}

/**************************************************************************/
/*!
    @brief  Enable debug messages on the chosen serial port
            Calling this function with nothing sets the debug port to Serial
            You can also call it with other streams like Serial1, SerialUSB, etc.
    @param  debugPort
            The serial port to be used for the debug messages
*/
/**************************************************************************/
void SWARM_M138::enableDebugging(Stream &debugPort)
{
  _debugPort = &debugPort;
  _printDebug = true;
}

/**************************************************************************/
/*!
    @brief  Disable debug messages
*/
/**************************************************************************/
void SWARM_M138::disableDebugging(void)
{
  _printDebug = false;
}

/**************************************************************************/
/*!
    @brief  Check for the arrival of new serial data. Parse it.
            Process any unsolicited messages. Call the callback(s) - if enabled.
    @return True if at least one unsolicited message was processed, otherwise false
*/
/**************************************************************************/
bool SWARM_M138::checkUnsolicitedMsg(void)
{
  if (_checkUnsolicitedMsgReentrant == true) // Check for reentry (i.e. checkUnsolicitedMsg has been called from inside a callback)
    return false;

  _checkUnsolicitedMsgReentrant = true;

  size_t avail = 0; // The number of available serial bytes
  bool handled = false; // Flag if any unsolicited messages were handled
  unsigned long timeIn = millis(); // Record the time so we can timeout
  char *event; // Each unsolicited messages is an 'event'

  memset(_swarmRxBuffer, 0, _RxBuffSize); // Clear _swarmRxBuffer

  // Does the backlog contain any data? If it does, copy it into _swarmRxBuffer and then clear the backlog
  // All of the serial data from the modem is 'printable'. It should never contain a \0. So it is OK to use strlen.
  size_t backlogLength = strlen((const char *)_swarmBacklog);
  if (backlogLength > 0)
  {
    //The backlog also logs reads from other tasks like transmitting.
    if (_printDebug == true)
    {
      _debugPort->print(F("checkUnsolicitedMsg: backlog found! backlog length is "));
      _debugPort->println(backlogLength);
    }
    memcpy(_swarmRxBuffer + avail, _swarmBacklog, backlogLength); // avail is zero
    avail += backlogLength;
    memset(_swarmBacklog, 0, _RxBuffSize); // Clear the backlog making sure it is NULL-terminated
  }

  int hwAvail = hwAvailable();
  if ((hwAvail > 0) || (backlogLength > 0)) // If either new data is available, or backlog had data.
  {
    // Wait for up to _rxWindowMillis for new serial data to arrive. 
    while (((millis() - timeIn) < _rxWindowMillis) && ((avail + hwAvail) < _RxBuffSize))
    {
      if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        avail += hwReadChars((char *)&_swarmRxBuffer[avail], hwAvail);
        timeIn = millis();
      }
      hwAvail = hwAvailable();
    }

    // _swarmRxBuffer now contains the backlog (if any) and the new serial data (if any)

    // A health warning about strtok:
    // strtok will convert any delimiters it finds ("\n" in our case) into NULL characters.
    // Also, be very careful that you do not use strtok within an strtok while loop.
    // The next call of strtok(NULL, ...) in the outer loop will use the pointer saved from the inner loop!
    // In our case, strtok is also used in pruneBacklog, which is called by waitForRespone or sendCommandWithResponse,
    // which is called by the parse functions called by processURCEvent...
    // The solution is to use strtok_r - the reentrant version of strtok

    char *preservedEvent;
    event = strtok_r(_swarmRxBuffer, "\n", &preservedEvent); // Look for an 'event' (_swarmRxBuffer contains something ending in \n)

    if (event != NULL)
      if (_printDebug == true)
        _debugPort->println(F("checkUnsolicitedMsg: event(s) found! ===>"));

    while (event != NULL) // Keep going until all events have been processed
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("checkUnsolicitedMsg: start of event: "));
        _debugPort->println(event);
      }

      //Process the event
      bool latestHandled = processUnsolicitedEvent((const char *)event);
      if (latestHandled)
        handled = true; // handled will be true if latestHandled has ever been true

      backlogLength = strlen((const char *)_swarmBacklog);
      if ((backlogLength > 0) && ((avail + backlogLength) < _RxBuffSize)) // Has any new data been added to the backlog?
      {
        if (_printDebug == true)
        {
          _debugPort->println(F("checkUnsolicitedMsg: new backlog added!"));
        }
        memcpy(_swarmRxBuffer + avail, _swarmBacklog, backlogLength);
        avail += backlogLength;
        memset(_swarmBacklog, 0, _RxBuffSize); //Clear out the backlog buffer again.
      }

      //Walk through any remaining events
      event = strtok_r(NULL, "\n", &preservedEvent);

      if (_printDebug == true)
        _debugPort->println(F("checkUnsolicitedMsg: end of event")); //Just to denote end of processing event.

      if (event == NULL)
        if (_printDebug == true)
          _debugPort->println(F("checkUnsolicitedMsg: <=== end of event(s)!"));
    }
  }

  free(event);

  _checkUnsolicitedMsgReentrant = false;

  return handled;
} // /checkUnsolicitedMsg

// Parse incoming unsolicited messages - pass the data to the user via the callbacks (if defined)
bool SWARM_M138::processUnsolicitedEvent(const char *event)
{
  // { // URC: +UULOC (Localization information - CellLocate and hybrid positioning)
  //   ClockData clck;
  //   PositionData gps;
  //   SpeedData spd;
  //   unsigned long uncertainty;
  //   int scanNum;
  //   int latH, lonH, alt;
  //   unsigned int speedU, cogU;
  //   char latL[10], lonL[10];
  //   int dateStore[5];

  //   // Maybe we should also scan for +UUGIND and extract the activated gnss system?

  //   // This assumes the ULOC response type is "0" or "1" - as selected by gpsRequest detailed
  //   scanNum = sscanf(event,
  //                     "+UULOC: %d/%d/%d,%d:%d:%d.%d,%d.%[^,],%d.%[^,],%d,%lu,%u,%u,%*s",
  //                     &dateStore[0], &dateStore[1], &clck.date.year,
  //                     &dateStore[2], &dateStore[3], &dateStore[4], &clck.time.ms,
  //                     &latH, latL, &lonH, lonL, &alt, &uncertainty,
  //                     &speedU, &cogU);
  //   clck.date.day = dateStore[0];
  //   clck.date.month = dateStore[1];
  //   clck.time.hour = dateStore[2];
  //   clck.time.minute = dateStore[3];
  //   clck.time.second = dateStore[4];

  //   if (scanNum >= 13)
  //   {
  //     // Found a Location string!
  //     if (_printDebug == true)
  //     {
  //       _debugPort->println(F("processReadEvent: location"));
  //     }

  //     if (latH >= 0)
  //       gps.lat = (float)latH + ((float)atol(latL) / pow(10, strlen(latL)));
  //     else
  //       gps.lat = (float)latH - ((float)atol(latL) / pow(10, strlen(latL)));
  //     if (lonH >= 0)
  //       gps.lon = (float)lonH + ((float)atol(lonL) / pow(10, strlen(lonL)));
  //     else
  //       gps.lon = (float)lonH - ((float)atol(lonL) / pow(10, strlen(lonL)));
  //     gps.alt = (float)alt;
  //     if (scanNum >= 15) // If detailed response, get speed data
  //     {
  //       spd.speed = (float)speedU;
  //       spd.cog = (float)cogU;
  //     }

  //     // if (_printDebug == true)
  //     // {
  //     //   _debugPort->print(F("processReadEvent: location:  lat: "));
  //     //   _debugPort->print(gps.lat, 7);
  //     //   _debugPort->print(F(" lon: "));
  //     //   _debugPort->print(gps.lon, 7);
  //     //   _debugPort->print(F(" alt: "));
  //     //   _debugPort->print(gps.alt, 2);
  //     //   _debugPort->print(F(" speed: "));
  //     //   _debugPort->print(spd.speed, 2);
  //     //   _debugPort->print(F(" cog: "));
  //     //   _debugPort->println(spd.cog, 2);
  //     // }

  //     if (_gpsRequestCallback != NULL)
  //     {
  //       _gpsRequestCallback(clck, gps, spd, uncertainty);
  //     }

  //     return true;
  //   }
  // }
  return false;
} // /processUnsolicitedEvent

/**************************************************************************/
/*!
    @brief  Read the modem device ID and name using the $CS message
    @param  settings
            A pointer to where the settings will be stored
            It is recommended that the user allocates at least
            SWARM_M138_MEM_ALLOC_CS bytes to store the settings
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
// Only 25 bytes are needed to store the reply,
// but an unsolicited receive data message could arrive while we are waiting for the response...
Swarm_M138_Error_e SWARM_M138::getConfigurationSettings(char *settings)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_calloc_char(strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5); // Clear it
  sprintf(command, "%s*", SWARM_M138_COMMAND_CONFIGURATION); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_calloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    free(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$CS DI=0x", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$CS DI=0x");
    if (responseStart == NULL)
    {
      free(command);
      free(response);
      return (SWARM_M138_ERROR_ERROR);
    }
    responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if (responseEnd == NULL)
    {
      free(command);
      free(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Copy the response to *settings
    responseStart += 4; // Start at the 'D'
    memcpy(settings, responseStart, responseEnd - responseStart);

    // Add a null-terminator
    settings[responseEnd - responseStart] = 0;
  }

  free(command);
  free(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Read the modem device ID using the $CS message
    @param  id
            The address of the uint32_t where the ID will be stored
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
// Only ~25 bytes are needed to store the reply,
// but an unsolicited receive data message _could_ arrive while we are waiting for the response...
// So we need to allocate the full _RxBuffSize for the response.
Swarm_M138_Error_e SWARM_M138::getDeviceID(uint32_t *id)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd;
  Swarm_M138_Error_e err;
  uint32_t dev_ID = 0;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_calloc_char(strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5); // Clear it
  sprintf(command, "%s*", SWARM_M138_COMMAND_CONFIGURATION); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_calloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    free(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$CS DI=0x", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$CS DI=0x");
    if (responseStart == NULL)
    {
      free(command);
      free(response);
      return (SWARM_M138_ERROR_ERROR);
    }
    responseEnd = strchr(responseStart, ','); // Stop at the comma
    if (responseEnd == NULL)
    {
      free(command);
      free(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the ID
    responseStart += 9; // Point at the first digit
    while (responseStart < responseEnd)
    {
      dev_ID <<= 4; // Shuffle the existing value along by 4 bits
      char c = *responseStart; // Get the digit
      if ((c >= '0') && (c <= '9'))
        dev_ID |= c - '0';
      else if ((c >= 'a') && (c <= 'f'))
        dev_ID |= c + 10 - 'a';
      if ((c >= 'A') && (c <= 'F'))
        dev_ID |= c + 10 - 'A';
      responseStart++;
    }

    if (_printDebug == true)
    {
      _debugPort->print(F("getDeviceID: dev_ID is 0x"));
      _debugPort->println(dev_ID, HEX);
    }

    *id = dev_ID; // Copy the extracted ID into id
  }

  free(command);
  free(response);
  return (err);
}

// String SWARM_M138::clock(void)
// {
//   Swarm_M138_Error_e err;
//   char *command;
//   char *response;
//   char *clockBegin;
//   char *clockEnd;

//   command = sara_r5_calloc_char(strlen(SWARM_M138_COMMAND_CLOCK) + 2);
//   if (command == NULL)
//     return "";
//   sprintf(command, "%s?", SWARM_M138_COMMAND_CLOCK);

//   response = sara_r5_calloc_char(minimumResponseAllocation);
//   if (response == NULL)
//   {
//     free(command);
//     return "";
//   }

//   err = sendCommandWithResponse(command, SWARM_M138_RESPONSE_OK,
//                                 response, SWARM_M138_STANDARD_RESPONSE_TIMEOUT);
//   if (err != SWARM_M138_ERROR_SUCCESS)
//   {
//     free(command);
//     free(response);
//     return "";
//   }

//   // Response format: \r\n+CCLK: "YY/MM/DD,HH:MM:SS-TZ"\r\n\r\nOK\r\n
//   clockBegin = strchr(response, '\"'); // Find first quote
//   if (clockBegin == NULL)
//   {
//     free(command);
//     free(response);
//     return "";
//   }
//   clockBegin += 1;                     // Increment pointer to begin at first number
//   clockEnd = strchr(clockBegin, '\"'); // Find last quote
//   if (clockEnd == NULL)
//   {
//     free(command);
//     free(response);
//     return "";
//   }
//   *(clockEnd) = '\0'; // Set last quote to null char -- end string

//   String clock = String(clockBegin); // Extract the clock as a String _before_ freeing response

//   free(command);
//   free(response);

//   return (clock);
// }

/**************************************************************************/
/*!
    @brief  Set up the callback for the $DT Date Time message
    @param  _swarmDateTimeCallback
            The address of the function to be called when an unsolicited $DT message arrives
*/
/**************************************************************************/
void SWARM_M138::setDateTimeCallback(void (*swarmDateTimeCallback)(const Swarm_M138_DateTimeData_t *dateTime))
{
  _swarmDateTimeCallback = swarmDateTimeCallback;
}

/**************************************************************************/
/*!
    @brief  Convert modem status enum into printable text
    @param  status
            The modem status (enumerated)
    @return A pointer to the modem status in string (const char) format
*/
/**************************************************************************/
const char *SWARM_M138::modemStatusString(Swarm_M138_Modem_Status_e status)
{
  switch (status)
  {
    case SWARM_M138_MODEM_STATUS_BOOT_ABORT:
      return "BOOT ABORT (Restart after firmware crash)";
      break;
    case SWARM_M138_MODEM_STATUS_BOOT_POWERON:
      return "BOOT POWERON (Power has been applied)";
      break;
    case SWARM_M138_MODEM_STATUS_BOOT_RUNNING:
      return "BOOT RUNNING (Boot has completed. Ready to accept commands)";
      break;
    case SWARM_M138_MODEM_STATUS_BOOT_UPDATED:
      return "BOOT UPDATED (A firmware update was performed)";
      break;
    case SWARM_M138_MODEM_STATUS_BOOT_VERSION:
      return "BOOT VERSION (Firmware version)";
      break;
    case SWARM_M138_MODEM_STATUS_DATETIME:
      return "DATETIME (GPS has acquired a valid date/time reference)";
      break;
    case SWARM_M138_MODEM_STATUS_POSITION:
      return "POSITION (GPS has acquired a valid position 3D fix)";
      break;
    case SWARM_M138_MODEM_STATUS_DEBUG:
      return "DEBUG (Debug message): ";
      break;
    case SWARM_M138_MODEM_STATUS_ERROR:
      return "ERROR (Error message): ";
      break;
  }
  return "UNKNOWN";
}

/**************************************************************************/
/*!
    @brief  Convert modem error enum into printable text
    @param  error
            The modem error (enumerated)
    @return A pointer to the modem error in string (const char) format
*/
/**************************************************************************/
const char *SWARM_M138::modemErrorString(Swarm_M138_Error_e error)
{
  switch (error)
  {
    case SWARM_M138_ERROR_ERROR:
      return "Just a plain old communication error";
      break;
    case SWARM_M138_ERROR_SUCCESS:
      return "Hey, it worked!";
      break;
    case SWARM_M138_ERROR_MEM_ALLOC:
      return "Memory allocation error";
      break;
    case SWARM_M138_ERROR_TIMEOUT:
      return "Communication timeout";
      break;
    case SWARM_M138_ERROR_INVALID_CHECKSUM:
      return "Indicates the command response checksum was invalid";
      break;
    case SWARM_M138_ERROR_ERR:
      return "Command input error (ERR)";
      break;
    case SWARM_M138_ERROR_MM_BADPARAM:
      return "Messages Received Management : invalid command or argument";
      break;
    case SWARM_M138_ERROR_MM_DBXINVMSGID:
      return "Messages Received Management : invalid message ID";
      break;
    case SWARM_M138_ERROR_MM_DBXNOMORE:
      return "Messages Received Management : no messages found";
      break;
    case SWARM_M138_ERROR_MT_BADPARAM:
      return "Messages To Transmit Management : invalid command or argument";
      break;
    case SWARM_M138_ERROR_MT_DBXINVMSGID:
      return "Messages To Transmit Management : invalid message ID";
      break;
    case SWARM_M138_ERROR_MT_DBXNOMORE:
      return "Messages To Transmit Management : no messages found";
      break;
    case SWARM_M138_ERROR_SL_TIMENOTSET:
      return "Sleep Mode : time not yet set from GPS";
      break;
    case SWARM_M138_ERROR_SL_BADPARAM:
      return "Sleep Mode : invalid seconds / dateTime";
      break;
    case SWARM_M138_ERROR_SL_NOCOMMAND:
      return "Sleep Mode : No S or U partameter";
      break;
    case SWARM_M138_ERROR_SL_NOTIME:
      return "Sleep Mode : attempt to sleep before time is set";
      break;
    case SWARM_M138_ERROR_TD_BADAPPID:
      return "Transmit Data : invalid application ID";
      break;
    case SWARM_M138_ERROR_TD_BADDATA:
      return "Transmit Data : Message has odd number or non-hex characters when sending data as hexadecimal";
      break;
    case SWARM_M138_ERROR_TD_BADEXPIRETIME:
      return "Transmit Data : Invalid hold time";
      break;
    case SWARM_M138_ERROR_TD_ERR:
      return "Transmit Data : Unspecified error";
      break;
    case SWARM_M138_ERROR_TD_HOLDTIMEEXPIRED:
      return "Transmit Data : Unable to send within requested hold time";
      break;
    case SWARM_M138_ERROR_TD_NODEVICEID:
      return "Transmit Data : The Swarm device ID has not yet been set - contact Swarm Support";
      break;
    case SWARM_M138_ERROR_TD_NOSPACE:
      return "Transmit Data : No space for message";
      break;
    case SWARM_M138_ERROR_TD_TIMENOTSET:
      return "Transmit Data : Attempt to send message before time set by GPS";
      break;
    case SWARM_M138_ERROR_TD_DBXTOHIVEFULL:
      return "Transmit Data : Queue for queued messages is full. Maximum of 2048 messages may be held in the queue.";
      break;
    case SWARM_M138_ERROR_TD_TOOLONG:
      return "Transmit Data : Message is too large to send";
      break;
  }

  return "UNKNOWN";
}

/////////////
// Private //
/////////////

// Add the two NMEA checksum bytes and line feed to a command
void SWARM_M138::addChecksumLF(char *command)
{
  char *dollar = strchr(command, '$'); // Find the $

  if (dollar == NULL) // Return now if the $ was not found
    return;

  char *asterix = strchr(command, '*'); // Find the *

  if (asterix == NULL) // Return now if the * was not found
    return;

  char checksum = 0;

  dollar++; // Point to the char after the $

  while (dollar < asterix) // Calculate the checksum
  {
    checksum ^= *dollar;
    dollar++;
  }

  // Add the checksum bytes to the command
  *(asterix + 1) = (checksum >> 4) + '0';
  if (*(asterix + 1) >= ':') // Hex a-f
    *(asterix + 1) = *(asterix + 1) + 'a' - ':';
  *(asterix + 2) = (checksum & 0x0F) + '0';
  if (*(asterix + 2) >= ':') // Hex a-f
    *(asterix + 2) = *(asterix + 2) + 'a' - ':';

  // Add the line feed
  *(asterix + 3) = '\n';

  // Add a \0 - just in case
  *(asterix + 4) = 0;
}


// Send a command. Check for a response.
// Return true if expectedResponseStart is seen in the data followed by a \n
Swarm_M138_Error_e SWARM_M138::sendCommandWithResponse(
    const char *command, const char *expectedResponseStart, char *responseDest,
    int destSize, unsigned long commandTimeout)
{
  bool found = false;
  bool responseStartSeen = false;
  int index = 0;
  int destIndex = 0;

  bool printedSomething = false;

  if (_printDebug == true)
  {
    _debugPort->print(F("sendCommandWithResponse: Command: "));
    _debugPort->println(command);
  }

  sendCommand(command); //Sending command needs to dump data to backlog buffer as well.
  unsigned long timeIn = millis();

  while ((!found) && ((timeIn + commandTimeout) > millis()))
  {
    int hwAvail = hwAvailable();
    if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      if ((destIndex + hwAvail) <= destSize) // Check there is room to store the response
      {
        int bytesRead = hwReadChars((char *)&responseDest[destIndex], hwAvail);

        if (_printDebug == true)
        {
          if (printedSomething == false)
          {
            _debugPort->print(F("sendCommandWithResponse: Response: "));
            printedSomething = true;
          }
          _debugPort->print((const char *)&responseDest[destIndex]);
        }
        
        // Check each character to see if it is the expected resonse or error
        for (size_t chrPtr = destIndex; chrPtr < (destIndex + bytesRead); chrPtr++)
        {
          char c = responseDest[chrPtr]; // Check each character
          if (c == expectedResponseStart[index])
          {
            if (++index == (int)strlen(expectedResponseStart))
            {
              responseStartSeen = true;
            }
          }
          else
          {
            index = 0;
          }
          if ((responseStartSeen) && (c == '\n'))
            found = true;
        }

        // Now also copy the response into the backlog, if there is room
        size_t backlogLength = strlen((const char *)_swarmBacklog);

        if ((backlogLength + bytesRead) <= _RxBuffSize) // Is there room to store the new data?
        {
          memcpy((char *)&_swarmBacklog[backlogLength], (char *)&responseDest[destIndex], bytesRead);
        }
        else
        {
          if (_printDebug == true)
          {
            if (printedSomething == true)
            {
              _debugPort->println();
              printedSomething = false;
            }
            _debugPort->println(F("sendCommandWithResponse: Panic! _swarmBacklog is full!"));
          }
        }

        destIndex += bytesRead; // Increment destIndex
      }
      else
      {
        if (_printDebug == true)
        {
          if (printedSomething == true)
          {
            _debugPort->println();
            printedSomething = false;
          }
          _debugPort->println(F("sendCommandWithResponse: Panic! responseDest is full!"));
        }
      }
    }
  }

  if (_printDebug == true)
    if (printedSomething)
      _debugPort->println();

  pruneBacklog(); // Prune any incoming non-actionable URC's and responses/errors from the backlog

  if (found)
  {
    return SWARM_M138_ERROR_SUCCESS;
  }

  return SWARM_M138_ERROR_TIMEOUT;
}

void SWARM_M138::sendCommand(const char *command)
{
  //Spend up to _rxWindowMillis milliseconds copying any incoming serial data into the backlog
  unsigned long timeIn = millis();
  int hwAvail = hwAvailable();
  if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
  {
    size_t backlogLength = strlen((const char *)_swarmBacklog);
    while (((millis() - timeIn) < _rxWindowMillis) && ((backlogLength + hwAvail) <= _RxBuffSize)) //May need to escape on newline?
    {
      if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        backlogLength += hwReadChars((char *)&_swarmBacklog[backlogLength], hwAvail);
        timeIn = millis();
      }
      hwAvail = hwAvailable();
    }
  }

  //Now send the command
  hwPrint(command);
}

Swarm_M138_Error_e SWARM_M138::waitForResponse(const char *expectedResponse, const char *expectedError, unsigned long timeout)
{
  unsigned long timeIn;
  bool found = false;
  int responseIndex = 0, errorIndex = 0;

  bool printedSomething = false;

  timeIn = millis();

  while ((!found) && ((timeIn + timeout) > millis()))
  {
    int hwAvail = hwAvailable();
    if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      // Store everything in the backlog - if there is room
      // _swarmBacklog is a global array that holds the backlog of any events
      // that came in while waiting for response. To be processed later within checkUnsolicitedMsg().
      // Note: the expectedResponse or expectedError will also be added to the backlog.
      // Everything in the backlog is 'printable'. So it is OK to use strlen.
      size_t backlogLength = strlen((const char *)_swarmBacklog);

      if ((backlogLength + hwAvail) <= _RxBuffSize) // Is there room to store the new data?
      {
        hwReadChars((char *)&_swarmBacklog[backlogLength], hwAvail);

        if (_printDebug == true)
        {
          if (printedSomething == false)
          {
            _debugPort->print(F("waitForResponse: "));
            printedSomething = true;
          }
          _debugPort->print((const char *)&_swarmBacklog[backlogLength]);
        }

        // Check each character to see if it is the expected resonse or error
        for (size_t chrPtr = backlogLength; chrPtr < (backlogLength + hwAvail); chrPtr++)
        {
          char c = _swarmBacklog[chrPtr]; // Check each character
          if (c == expectedResponse[responseIndex])
          {
            if (++responseIndex == (int)strlen(expectedResponse))
            {
              found = true;
            }
          }
          else
          {
            responseIndex = 0;
          }
          if (expectedError != NULL)
          {
            if (c == expectedError[errorIndex])
            {
              if (++errorIndex == (int)strlen(expectedError))
              {
                found = true;
              }
            }
            else
            {
              errorIndex = 0;
            }
          }
        }
      }
      else
      {
        if (_printDebug == true)
        {
          if (printedSomething == true)
          {
            _debugPort->println();
            printedSomething = false;
          }
          _debugPort->println(F("waitForResponse: Panic! _swarmBacklog is full!"));
        }
      }
    }
  }

  if (_printDebug == true)
    if (printedSomething)
      _debugPort->println();

  pruneBacklog(); // Prune any incoming non-actionable URC's and responses/errors from the backlog

  if (found == true)
  {
    if (responseIndex > 0) // Let success have priority
    {
      return SWARM_M138_ERROR_SUCCESS;
    }
    else if (errorIndex > 0)
    {
      return SWARM_M138_ERROR_ERROR;
    }
  }

  return SWARM_M138_ERROR_TIMEOUT;
}

// Swarm_M138_Error_e SWARM_M138::parseSocketReadIndication(int socket, int length)
// {
//   Swarm_M138_Error_e err;
//   char *readDest;

//   if ((socket < 0) || (length < 0))
//   {
//     return SWARM_M138_ERROR_UNEXPECTED_RESPONSE;
//   }

//   // Return now if both callbacks pointers are NULL - otherwise the data will be read and lost!
//   if ((_socketReadCallback == NULL) && (_socketReadCallbackPlus == NULL))
//     return SWARM_M138_ERROR_INVALID;

//   readDest = sara_r5_calloc_char(length + 1);
//   if (readDest == NULL)
//     return SWARM_M138_ERROR_OUT_OF_MEMORY;

//   int bytesRead;
//   err = socketRead(socket, length, readDest, &bytesRead);
//   if (err != SWARM_M138_ERROR_SUCCESS)
//   {
//     free(readDest);
//     return err;
//   }

//   if (_socketReadCallback != NULL)
//   {
//     String dataAsString = ""; // Create an empty string
//     // Copy the data from readDest into the String in a binary-compatible way
//     // Important Note: some implementations of concat, like the one on ESP32, are binary-compatible.
//     // But some, like SAMD, are not. They use strlen or strcpy internally - which don't like \0's.
//     // The only true binary-compatible solution is to use socketReadCallbackPlus...
//     for (int i = 0; i < bytesRead; i++)
//       dataAsString.concat(readDest[i]);
//     _socketReadCallback(socket, dataAsString);
//   }

//   if (_socketReadCallbackPlus != NULL)
//   {
//     IPAddress dummyAddress = { 0, 0, 0, 0 };
//     int dummyPort = 0;
//     _socketReadCallbackPlus(socket, (const char *)readDest, bytesRead, dummyAddress, dummyPort);
//   }

//   free(readDest);
//   return SWARM_M138_ERROR_SUCCESS;
// }

size_t SWARM_M138::hwPrint(const char *s)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->print(s);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->print(s);
  }
#endif
  else if (_i2cPort != NULL)
  {
    return ((size_t)qwiicSwarmWriteChars(strlen(s), s));
  }

  return (size_t)0;
}

size_t SWARM_M138::hwWriteData(const char *buff, int len)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->write((const uint8_t *)buff, len);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->write((const uint8_t *)buff, len);
  }
#endif
  else if (_i2cPort != NULL)
  {
    return ((size_t)qwiicSwarmWriteChars(len, buff));
  }

  return (size_t)0;
}

size_t SWARM_M138::hwWrite(const char c)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->write(c);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->write(c);
  }
#endif
  else if (_i2cPort != NULL)
  {
    return ((size_t)qwiicSwarmWriteChars((int)1, &c));
  }

  return (size_t)0;
}

int SWARM_M138::hwAvailable(void)
{
  if (_hardSerial != NULL)
  {
    return ((int)_hardSerial->available());
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return ((int)_softSerial->available());
  }
#endif
  else if (_i2cPort != NULL)
  {
    return (qwiicSwarmAvailable());
  }

  return -1;
}

// Read len chars from the appropriate port. Store in buf
int SWARM_M138::hwReadChars(char *buf, int len)
{
  if (len <= 0)
    return (len);

  if (buf == NULL)
    return (-1);

  if (_hardSerial != NULL)
  {
    for (int i = 0; i < len; i++)
    {
      buf[i] = _hardSerial->read();
    }
    return (len);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    for (int i = 0; i < len; i++)
    {
      buf[i] = _softSerial->read();
    }
    return (len);
  }
#endif
  else if (_i2cPort != NULL)
  {
    return (qwiicSwarmReadChars(len, buf));
  }

  return (-1);

}

// I2C functions for Qwiic Swarm

// Check how many bytes Qwiic Swarm has available
// Return -1 if it is less than QWIIC_SWARM_I2C_POLLING_WAIT_MS since the last check
int SWARM_M138::qwiicSwarmAvailable(void)
{
  int bytesAvailable = -1;

  if (millis() - _lastI2cCheck >= QWIIC_SWARM_I2C_POLLING_WAIT_MS)
  {
    //Check how many serial bytes are waiting to be read
    _i2cPort->beginTransmission((uint8_t)_address); // Talk to the I2C device
    _i2cPort->write(QWIIC_SWARM_LEN_REG); // Point to the serial buffer length
    _i2cPort->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Controller holds the bus!)
    if (_i2cPort->requestFrom((uint8_t)_address, (uint8_t)2) == 2) // Request two bytes
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      bytesAvailable = (((uint16_t)msb) << 8) | lsb;
    }

    //Put off checking to avoid excessive I2C bus traffic - but only if zero bytes are available
    if (bytesAvailable == 0)
      _lastI2cCheck = millis();
  }

  return (bytesAvailable);
}

// Read len bytes from Qwiic Swarm, store in dest
int SWARM_M138::qwiicSwarmReadChars(int len, char *dest)
{
  if (len <= 0)
    return (len);

  if (dest == NULL)
    return (0);

  int bytesRead = 0;

  // Request the bytes
  // Release the bus afterwards
  _i2cPort->beginTransmission((uint8_t)_address); // Talk to the I2C device
  _i2cPort->write(QWIIC_SWARM_DATA_REG); // Point to the serial buffer
  _i2cPort->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
  while (len > QWIIC_SWARM_SER_PACKET_SIZE) // If there are _more_ than SER_PACKET_SIZE bytes to be read
  {
    _i2cPort->requestFrom((uint8_t)_address, (uint8_t)QWIIC_SWARM_SER_PACKET_SIZE, (uint8_t)false); // Request SER_PACKET_SIZE bytes, don't release the bus
    while (_i2cPort->available())
    {
      dest[bytesRead] = _i2cPort->read(); // Read and store each byte
      bytesRead++;
    }
    len -= QWIIC_SWARM_SER_PACKET_SIZE; // Decrease the number of bytes available by SER_PACKET_SIZE
  }
  _i2cPort->requestFrom((uint8_t)_address, (uint8_t)len); // Request remaining bytes, release the bus
  while (_i2cPort->available())
  {
    dest[bytesRead] = _i2cPort->read(); // Read and store each byte
    bytesRead++;
  }

  return (bytesRead);
}

// Write serial data to Qwiic Swarm
int SWARM_M138::qwiicSwarmWriteChars(int len, const char *dest)
{
  if (len <= 0)
    return (len);

  if (dest == NULL)
    return (0);

  size_t i = 0;
  size_t nexti;
  uint16_t checksum = 0;

  while (len > (QWIIC_SWARM_I2C_BUFFER_LENGTH - 3)) // If there are too many bytes to send all in one go
  {
    nexti = i + (QWIIC_SWARM_I2C_BUFFER_LENGTH - 3);
    _i2cPort->beginTransmission((uint8_t)_address);
    _i2cPort->write(QWIIC_SWARM_DATA_REG); // Point to the serial data 'register'
    for (; i < nexti; i++)
    {
        _i2cPort->write(dest[i]); // Write each byte
        checksum += (uint16_t)dest[i]; // Update the checksum
    }
    len -= (QWIIC_SWARM_I2C_BUFFER_LENGTH - 3); // Decrease the number of bytes still to send
    _i2cPort->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
  }

  // There are now <= (TINY_I2C_BUFFER_LENGTH - 3) bytes left to send, so send them and then release the bus
  _i2cPort->beginTransmission((uint8_t)_address);
  _i2cPort->write(QWIIC_SWARM_DATA_REG); // Point to the 'serial register'
  while (len > 0)
  {
    _i2cPort->write(dest[i]);
    checksum += (uint16_t)dest[i];
    len--;
    i++;
  }
  _i2cPort->write((uint8_t)(checksum >> 8));
  _i2cPort->write((uint8_t)(checksum & 0xFF));
  if (_i2cPort->endTransmission() != 0) //Send data and release bus
    if (_printDebug == true)
      _debugPort->println(F("qwiicSwarmWriteChars: I2C write was not successful!"));

  return ((int)i);
}

void SWARM_M138::beginSerial(unsigned long baud)
{
  if (_hardSerial != NULL)
  {
    _hardSerial->begin(baud);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    _softSerial->end();
    _softSerial->begin(baud);
  }
#endif
  delay(100);
}

// Allocate memory
char *SWARM_M138::swarm_m138_calloc_char(size_t num)
{
  return (char *)calloc(num, sizeof(char));
}

//This prunes the backlog of non-actionable events. If new actionable events are added, you must modify the if statement.
void SWARM_M138::pruneBacklog()
{
  char *event;

  memset(_pruneBuffer, 0, _RxBuffSize); // Clear the _pruneBuffer

  char *preservedEvent;
  event = strtok_r(_swarmBacklog, "\n", &preservedEvent); // Look for an 'event' - something ending in \n

  while (event != NULL) //If event is actionable, add it to pruneBuffer.
  {
    // These are the events we want to keep so they can be processed by poll / checkUnsolicitedMsg
    if ((strstr(event, "+UUSORD:") != NULL)
        || (strstr(event, "+UUSORF:") != NULL)
        || (strstr(event, "+UUSOLI:") != NULL)
        || (strstr(event, "+UUSOCL:") != NULL)
        || (strstr(event, "+UULOC:") != NULL)
        || (strstr(event, "+UUSIMSTAT:") != NULL)
        || (strstr(event, "+UUPSDA:") != NULL)
        || (strstr(event, "+UUPING:") != NULL)
        || (strstr(event, "+UUHTTPCR:") != NULL))
    {
      strcat(_pruneBuffer, event); // The URCs are all readable text so using strcat is OK
      strcat(_pruneBuffer, "\n"); // strtok blows away delimiter, but we want that for later.
    }

    event = strtok_r(NULL, "\n", &preservedEvent); // Walk though any remaining events
  }

  memset(_swarmBacklog, 0, _RxBuffSize); //Clear out backlog buffer.
  memcpy(_swarmBacklog, _pruneBuffer, strlen(_pruneBuffer)); //Copy the pruned buffer back into the backlog

  free(event);
}

// // GPS Helper Functions:

// // Read a source string until a delimiter is hit, store the result in destination
// char *SWARM_M138::readDataUntil(char *destination, unsigned int destSize,
//                              char *source, char delimiter)
// {

//   char *strEnd;
//   size_t len;

//   strEnd = strchr(source, delimiter);

//   if (strEnd != NULL)
//   {
//     len = strEnd - source;
//     memset(destination, 0, destSize);
//     memcpy(destination, source, len);
//   }

//   return strEnd;
// }

// bool SWARM_M138::parseGPRMCString(char *rmcString, PositionData *pos,
//                                ClockData *clk, SpeedData *spd)
// {
//   char *ptr, *search;
//   unsigned long tTemp;
//   char tempData[TEMP_NMEA_DATA_SIZE];

//   // if (_printDebug == true)
//   // {
//   //   _debugPort->println(F("parseGPRMCString: rmcString: "));
//   //   _debugPort->println(rmcString);
//   // }

//   // Fast-forward test to first value:
//   ptr = strchr(rmcString, ',');
//   ptr++; // Move ptr past first comma

//   // If the next character is another comma, there's no time data
//   // Find time:
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   // Next comma should be present and not the next position
//   if ((search != NULL) && (search != ptr))
//   {
//     pos->utc = atof(tempData);                             // Extract hhmmss.ss as float
//     tTemp = pos->utc;                                      // Convert to unsigned long (discard the digits beyond the decimal point)
//     clk->time.ms = ((unsigned int)(pos->utc * 100)) % 100; // Extract the milliseconds
//     clk->time.hour = tTemp / 10000;
//     tTemp -= ((unsigned long)clk->time.hour * 10000);
//     clk->time.minute = tTemp / 100;
//     tTemp -= ((unsigned long)clk->time.minute * 100);
//     clk->time.second = tTemp;
//   }
//   else
//   {
//     pos->utc = 0.0;
//     clk->time.hour = 0;
//     clk->time.minute = 0;
//     clk->time.second = 0;
//   }
//   ptr = search + 1; // Move pointer to next value

//   // Find status character:
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   // Should be a single character: V = Data invalid, A = Data valid
//   if ((search != NULL) && (search == ptr + 1))
//   {
//     pos->status = *ptr; // Assign char at ptr to status
//   }
//   else
//   {
//     pos->status = 'X'; // Made up very bad status
//   }
//   ptr = search + 1;

//   // Find latitude:
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search != ptr))
//   {
//     pos->lat = atof(tempData);              // Extract ddmm.mmmmm as float
//     unsigned long lat_deg = pos->lat / 100; // Extract the degrees
//     pos->lat -= (float)lat_deg * 100.0;     // Subtract the degrees leaving only the minutes
//     pos->lat /= 60.0;                       // Convert minutes into degrees
//     pos->lat += (float)lat_deg;             // Finally add the degrees back on again
//   }
//   else
//   {
//     pos->lat = 0.0;
//   }
//   ptr = search + 1;

//   // Find latitude hemishpere
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search == ptr + 1))
//   {
//     if (*ptr == 'S')    // Is the latitude South
//       pos->lat *= -1.0; // Make lat negative
//   }
//   ptr = search + 1;

//   // Find longitude:
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search != ptr))
//   {
//     pos->lon = atof(tempData);              // Extract dddmm.mmmmm as float
//     unsigned long lon_deg = pos->lon / 100; // Extract the degrees
//     pos->lon -= (float)lon_deg * 100.0;     // Subtract the degrees leaving only the minutes
//     pos->lon /= 60.0;                       // Convert minutes into degrees
//     pos->lon += (float)lon_deg;             // Finally add the degrees back on again
//   }
//   else
//   {
//     pos->lon = 0.0;
//   }
//   ptr = search + 1;

//   // Find longitude hemishpere
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search == ptr + 1))
//   {
//     if (*ptr == 'W')    // Is the longitude West
//       pos->lon *= -1.0; // Make lon negative
//   }
//   ptr = search + 1;

//   // Find speed
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search != ptr))
//   {
//     spd->speed = atof(tempData); // Extract speed over ground in knots
//     spd->speed *= 0.514444;      // Convert to m/s
//   }
//   else
//   {
//     spd->speed = 0.0;
//   }
//   ptr = search + 1;

//   // Find course over ground
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search != ptr))
//   {
//     spd->cog = atof(tempData);
//   }
//   else
//   {
//     spd->cog = 0.0;
//   }
//   ptr = search + 1;

//   // Find date
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search != ptr))
//   {
//     tTemp = atol(tempData);
//     clk->date.day = tTemp / 10000;
//     tTemp -= ((unsigned long)clk->date.day * 10000);
//     clk->date.month = tTemp / 100;
//     tTemp -= ((unsigned long)clk->date.month * 100);
//     clk->date.year = tTemp;
//   }
//   else
//   {
//     clk->date.day = 0;
//     clk->date.month = 0;
//     clk->date.year = 0;
//   }
//   ptr = search + 1;

//   // Find magnetic variation in degrees:
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search != ptr))
//   {
//     spd->magVar = atof(tempData);
//   }
//   else
//   {
//     spd->magVar = 0.0;
//   }
//   ptr = search + 1;

//   // Find magnetic variation direction
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
//   if ((search != NULL) && (search == ptr + 1))
//   {
//     if (*ptr == 'W')       // Is the magnetic variation West
//       spd->magVar *= -1.0; // Make magnetic variation negative
//   }
//   ptr = search + 1;

//   // Find position system mode
//   // Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, A = Autonomous GNSS fix,
//   //                              D = Differential GNSS fix, F = RTK float, R = RTK fixed
//   search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, '*');
//   if ((search != NULL) && (search = ptr + 1))
//   {
//     pos->mode = *ptr;
//   }
//   else
//   {
//     pos->mode = 'X';
//   }
//   ptr = search + 1;

//   if (pos->status == 'A')
//   {
//     return true;
//   }
//   return false;
// }
