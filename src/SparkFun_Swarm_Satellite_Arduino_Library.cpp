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

#include "SparkFun_Swarm_Satellite_Arduino_Library.h"

SWARM_M138::SWARM_M138(void)
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
  _checkUnsolicitedMsgReentrant = false;
  _lastI2cCheck = millis();

  _swarmDateTimeCallback = NULL;
  _swarmGpsJammingCallback = NULL;
  _swarmGeospatialCallback = NULL;
  _swarmGpsFixQualityCallback = NULL;
  _swarmPowerStatusCallback = NULL;
  _swarmReceiveMessageCallback = NULL;
  _swarmReceiveTestCallback = NULL;
  _swarmSleepWakeCallback = NULL;
  _swarmModemStatusCallback = NULL;
  _swarmTransmitDataCallback = NULL;

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
  _swarmBacklog = new char[_RxBuffSize];
  if (_swarmBacklog == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("begin: not enough memory for _swarmBacklog!"));
    return false;
  }
  memset(_swarmBacklog, 0, _RxBuffSize);

  commandError = new char[SWARM_M138_MAX_CMD_ERROR_LEN];
  if (commandError == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("begin: not enough memory for commandError!"));
    swarm_m138_free_char(_swarmBacklog);
    return false;
  }
  memset(commandError, 0, SWARM_M138_MAX_CMD_ERROR_LEN);

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

  char *_swarmRxBuffer = swarm_m138_alloc_char(_RxBuffSize);
  if (_swarmRxBuffer == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("checkUnsolicitedMsg: not enough memory for _swarmRxBuffer!"));
    return false;
  }
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
      else
        delay(1);
      hwAvail = hwAvailable();
    }

    // _swarmRxBuffer now contains the backlog (if any) and the new serial data (if any)

    // A health warning about strtok:
    //
    // strtok will convert any delimiters it finds ("\n" in our case) into NULL characters.
    //
    // Also, be very careful that you do not use strtok within an strtok while loop.
    // The next call of strtok(NULL, ...) in the outer loop will use the pointer saved from the inner loop!
    // In our case, strtok is also used in pruneBacklog, which is called by waitForResponse or sendCommandWithResponse,
    // which is called by the parse functions called by processUnsolicitedEvent...
    // The solution is to use strtok_r - the reentrant version of strtok
    //
    // Also, if the string does not contain any delimiters, strtok will still return a pointer to the start of the string.
    // The entire string is considered the first token.

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

      if (checkChecksum(event) == SWARM_M138_ERROR_SUCCESS) // Check the checksum
      {
        //Process the event
        bool latestHandled = processUnsolicitedEvent((const char *)event);
        if (latestHandled)
          handled = true; // handled will be true if latestHandled has ever been true
      }
      else
      {
        if (_printDebug == true)
          _debugPort->println(F("checkUnsolicitedMsg: event is invalid!"));
      }

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

  swarm_m138_free_char(_swarmRxBuffer);
  delete event;

  _checkUnsolicitedMsgReentrant = false;

  return handled;
} // /checkUnsolicitedMsg

// Parse incoming unsolicited messages - pass the data to the user via the callbacks (if defined)
bool SWARM_M138::processUnsolicitedEvent(const char *event)
{
  { // $DT - Date/Time
    Swarm_M138_DateTimeData_t *dateTime = new Swarm_M138_DateTimeData_t;
    if (dateTime != NULL) // Check memory allocation was successful
    {
      char *eventStart;
      char *eventEnd;

      eventStart = strstr(event, "$DT ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 20)) // Check we have enough data
          {
            // Extract the Date, Time and flag
            int year, month, day, hour, minute, second;
            char valid;

            int ret = sscanf(eventStart, "$DT %4d%2d%2d%2d%2d%2d,%c*", &year, &month, &day, &hour, &minute, &second, &valid);

            if (ret == 7)
            {
              dateTime->YYYY = (uint16_t)year;
              dateTime->DD = (uint8_t)month;
              dateTime->MM = (uint8_t)day;
              dateTime->hh = (uint8_t)hour;
              dateTime->mm = (uint8_t)minute;
              dateTime->ss = (uint8_t)second;
              dateTime->valid = valid == 'V' ? 1 : 0;

              if (_swarmDateTimeCallback != NULL)
              {
                _swarmDateTimeCallback((const Swarm_M138_DateTimeData_t *)dateTime); // Call the callback
              }

              delete dateTime;
              return (true);
            }
          }
        }
      }
      delete dateTime;
    }
  }
  { // $GJ - jamming indication
    Swarm_M138_GPS_Jamming_Indication_t *jamming = new Swarm_M138_GPS_Jamming_Indication_t;
    if (jamming != NULL) // Check memory allocation was successful
    {
      char *eventStart;
      char *eventEnd;

      eventStart = strstr(event, "$GJ ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 3)) // Check we have enough data
          {
            // Extract the spoof_state and jamming_level
            int spoof_state, jamming_level;

            int ret = sscanf(eventStart, "$GJ %d,%d*", &spoof_state, &jamming_level);

            if (ret == 2)
            {
              jamming->spoof_state = (uint8_t)spoof_state;
              jamming->jamming_level = (uint8_t)jamming_level;

              if (_swarmGpsJammingCallback != NULL)
              {
                _swarmGpsJammingCallback((const Swarm_M138_GPS_Jamming_Indication_t *)jamming); // Call the callback
              }

              delete jamming;
              return (true);
            }
          }
        }
      }
      delete jamming;
    }
  }
  { // $GN - geospatial information
    Swarm_M138_GeospatialData_t *info = new Swarm_M138_GeospatialData_t;
    if (info != NULL) // Check memory allocation was successful
    {
      char *eventStart;
      char *eventEnd;

      eventStart = strstr(event, "$GN ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 10)) // Check we have enough data
          {
            // Extract the geospatial info
            int latH, lonH, alt, course, speed;
            char latL[8], lonL[8];

            int ret = sscanf(eventStart, "$GN %d.%[^,],%d.%[^,],%d,%d,%d*",
                             &latH, latL, &lonH, lonL, &alt, &course, &speed);

            if (ret == 7)
            {
              if (latH >= 0)
                info->lat = (float)latH + ((float)atol(latL) / pow(10, strlen(latL)));
              else
                info->lat = (float)latH - ((float)atol(latL) / pow(10, strlen(latL)));
              if (lonH >= 0)
                info->lon = (float)lonH + ((float)atol(lonL) / pow(10, strlen(lonL)));
              else
                info->lon = (float)lonH - ((float)atol(lonL) / pow(10, strlen(lonL)));
              info->alt = (float)alt;
              info->course = (float)course;
              info->speed = (float)speed;

              if (_swarmGeospatialCallback != NULL)
              {
                _swarmGeospatialCallback((const Swarm_M138_GeospatialData_t *)info); // Call the callback
              }

              delete info;
              return (true);
            }
          }
        }
      }
      delete info;
    }
  }
  { // $GS - GPS fix quality
    Swarm_M138_GPS_Fix_Quality_t *fixQuality = new Swarm_M138_GPS_Fix_Quality_t;
    if (fixQuality != NULL) // Check memory allocation was successful
    {
      char *eventStart;
      char *eventEnd;

      eventStart = strstr(event, "$GS ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 11)) // Check we have enough data
          {
            // Extract the GPS fix quality
            int hdop, vdop, gnss_sats, unused;
            char fix_type[3];

            int ret = sscanf(eventStart, "$GS %d,%d,%d,%d,%c%c*", &hdop, &vdop, &gnss_sats, &unused, &fix_type[0], &fix_type[1]);

            if (ret == 6)
            {
              fixQuality->hdop = (uint16_t)hdop;
              fixQuality->vdop = (uint16_t)vdop;
              fixQuality->gnss_sats = (uint8_t)gnss_sats;
              fixQuality->unused = (uint8_t)unused;

              fix_type[2] = 0; // Null-terminate the fix type
              if (strstr(fix_type, "NF") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_NF;
              else if (strstr(fix_type, "DR") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_DR;
              else if (strstr(fix_type, "G2") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_G2;
              else if (strstr(fix_type, "G3") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_G3;
              else if (strstr(fix_type, "D2") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_D2;
              else if (strstr(fix_type, "D3") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_D3;
              else if (strstr(fix_type, "RK") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_RK;
              else if (strstr(fix_type, "TT") != NULL)
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_TT;
              else
                fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_INVALID;

              if (_swarmGpsFixQualityCallback != NULL)
              {
                _swarmGpsFixQualityCallback((const Swarm_M138_GPS_Fix_Quality_t *)fixQuality); // Call the callback
              }

              delete fixQuality;
              return (true);
            }
          }
        }
      }
      delete fixQuality;
    }
  }
  { // $PW - Power Status
    Swarm_M138_Power_Status_t *powerStatus = new Swarm_M138_Power_Status_t;
    if (powerStatus != NULL) // Check memory allocation was successful
    {
      char *eventStart;
      char *eventEnd;

      eventStart = strstr(event, "$PW ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 10)) // Check we have enough data
          {
            // Extract the power status
            int unused1H, unused2H, unused3H, cpu_voltsH, tempH;
            char unused1L[8], unused2L[8], unused3L[8], cpu_voltsL[8], tempL[8];

            int ret = sscanf(eventStart, "$PW %d.%[^,],%d.%[^,],%d.%[^,],%d.%[^,],%d.%[^,]*",
                            &cpu_voltsH, cpu_voltsL, &unused1H, unused1L,
                            &unused2H, unused2L, &unused3H, unused3L,
                            &tempH, tempL);

            if (ret == 10)
            {
              if (cpu_voltsH >= 0)
                powerStatus->cpu_volts = (float)cpu_voltsH + ((float)atol(cpu_voltsL) / pow(10, strlen(cpu_voltsL)));
              else
                powerStatus->cpu_volts = (float)cpu_voltsH - ((float)atol(cpu_voltsL) / pow(10, strlen(cpu_voltsL)));
              if (unused1H >= 0)
                powerStatus->unused1 = (float)unused1H + ((float)atol(unused1L) / pow(10, strlen(unused1L)));
              else
                powerStatus->unused1 = (float)unused1H - ((float)atol(unused1L) / pow(10, strlen(unused1L)));
              if (unused2H >= 0)
                powerStatus->unused2 = (float)unused2H + ((float)atol(unused2L) / pow(10, strlen(unused2L)));
              else
                powerStatus->unused2 = (float)unused2H - ((float)atol(unused2L) / pow(10, strlen(unused2L)));
              if (unused3H >= 0)
                powerStatus->unused3 = (float)unused3H + ((float)atol(unused3L) / pow(10, strlen(unused3L)));
              else
                powerStatus->unused3 = (float)unused3H - ((float)atol(unused3L) / pow(10, strlen(unused3L)));
              if (tempH >= 0)
                powerStatus->temp = (float)tempH + ((float)atol(tempL) / pow(10, strlen(tempL)));
              else
                powerStatus->temp = (float)tempH - ((float)atol(tempL) / pow(10, strlen(tempL)));

              if (_swarmPowerStatusCallback != NULL)
              {
                _swarmPowerStatusCallback((const Swarm_M138_Power_Status_t *)powerStatus); // Call the callback
              }

              delete powerStatus;
              return (true);
            }
          }
        }
      }
      delete powerStatus;
    }
  }
  { // $RT - Receive Test
    Swarm_M138_Receive_Test_t *rxTest = new Swarm_M138_Receive_Test_t;
    if (rxTest != NULL) // Check memory allocation was successful
    {
      char *eventStart;
      char *eventEnd;

      eventStart = strstr(event, "$RT ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 9)) // Check we have enough data
          {
            // Extract the receive test info
            int rssi_bg = 0, rssi_sat = 0, snr = 0, fdev = 0;
            int YYYY = 0, MM = 0, DD = 0, hh = 0, mm = 0, ss = 0;
            uint32_t sat_ID = 0;

            int ret = sscanf(eventStart, "$RT RSSI=%d,SNR=%d,FDEV=%d,TS=%d-%d-%dT%d:%d:%d,DI=0x",
                            &rssi_sat, &snr, &fdev, &YYYY, &MM, &DD, &hh, &mm, &ss);

            if (ret == 9)
            {
              eventStart = strstr(eventStart, "DI=0x"); // Find the start of the satellite ID

              // Extract the ID
              eventStart += 5; // Point at the first digit
              while (eventStart < eventEnd)
              {
                sat_ID <<= 4; // Shuffle the existing value along by 4 bits
                char c = *eventStart; // Get the digit
                if ((c >= '0') && (c <= '9'))
                  sat_ID |= c - '0';
                else if ((c >= 'a') && (c <= 'f'))
                  sat_ID |= c + 10 - 'a';
                else if ((c >= 'A') && (c <= 'F'))
                  sat_ID |= c + 10 - 'A';
                eventStart++;
              }
            }
            else // Try to extract just rssi_background
            {
              ret = sscanf(eventStart, "$RT RSSI=%d*", &rssi_bg);
            }

            if ((ret == 9) || (ret == 1)) // Check if we got valid data
            {
              rxTest->background = ret == 1;
              rxTest->rssi_background = (int16_t)rssi_bg;
              rxTest->rssi_sat = (int16_t)rssi_sat;
              rxTest->snr = (int16_t)snr;
              rxTest->fdev = (int16_t)fdev;
              rxTest->time.YYYY = YYYY;
              rxTest->time.MM = MM;
              rxTest->time.DD = DD;
              rxTest->time.hh = hh;
              rxTest->time.mm = mm;
              rxTest->time.ss = ss;
              rxTest->sat_id = sat_ID;

              if (_swarmReceiveTestCallback != NULL)
              {
                _swarmReceiveTestCallback((const Swarm_M138_Receive_Test_t *)rxTest); // Call the callback
              }

              delete rxTest;
              return (true);
            }
          }
        }
      }
      delete rxTest;
    }
  }
  { // $M138 - Modem Status
    char *data = swarm_m138_alloc_char(SWARM_M138_MEM_ALLOC_MS);
    if (data != NULL) // Check memory allocation was successful
    {
      Swarm_M138_Modem_Status_e status = SWARM_M138_MODEM_STATUS_INVALID;
      char *eventStart;
      char *eventEnd;

      memset(data, 0, SWARM_M138_MEM_ALLOC_MS); // Clear the data

      eventStart = strstr(event, "$M138 ");
      if (eventStart != NULL)
      {
        eventEnd = strchr(eventStart, '*'); // Stop at the asterix
        if (eventEnd != NULL)
        {
          if (eventEnd >= (eventStart + 6)) // Check we have enough data
          {
            // Extract the modem status

            eventStart += 6; // Point at the first character of the msg

            if (strstr(eventStart, "BOOT,ABORT") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_ABORT;
              eventStart += strlen("BOOT,ABORT"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,DEVICEID") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_DEVICEID;
              eventStart += strlen("BOOT,DEVICEID"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,POWERON") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_POWERON;
              eventStart += strlen("BOOT,POWERON"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,RUNNING") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_RUNNING;
              eventStart += strlen("BOOT,RUNNING"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,UPDATED") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_UPDATED;
              eventStart += strlen("BOOT,UPDATED"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,VERSION") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_VERSION;
              eventStart += strlen("BOOT,VERSION"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,RESTART") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_RESTART;
              eventStart += strlen("BOOT,RESTART"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "BOOT,SHUTDOWN") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_BOOT_SHUTDOWN;
              eventStart += strlen("BOOT,SHUTDOWN"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "DATETIME") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_DATETIME;
              eventStart += strlen("DATETIME"); // Point at the asterix
            }
            else if (strstr(eventStart, "POSITION") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_POSITION;
              eventStart += strlen("POSITION"); // Point at the asterix
            }
            else if (strstr(eventStart, "DEBUG") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_DEBUG;
              eventStart += strlen("DEBUG"); // Point at the comma (or asterix)
            }
            else if (strstr(eventStart, "ERROR") != NULL)
            {
              status = SWARM_M138_MODEM_STATUS_ERROR;
              eventStart += strlen("ERROR"); // Point at the comma (or asterix)
            }

            if (*eventStart == ',') // Is eventStart pointing at a comma?
              eventStart++; // Point at the next character

            if (eventStart < eventEnd) // Check if we have reached the asterix
            {
              if (status == SWARM_M138_MODEM_STATUS_INVALID) // If status is still INVALID, this must be an unknown / undocumented message
                status = SWARM_M138_MODEM_STATUS_UNKNOWN;

              // Keep going until we hit the asterix or the data buffer is full
              // Leave a NULL on the end of data!
              size_t dataLen = 0;
              while ((eventStart < eventEnd) && (dataLen < (SWARM_M138_MEM_ALLOC_MS - 1)))
              {
                data[dataLen] = *eventStart; // Copy the message data into data
                dataLen++;
                eventStart++;
              }
            }

            if (status < SWARM_M138_MODEM_STATUS_INVALID) // Check if we got valid data
            {
              if (_swarmModemStatusCallback != NULL)
              {
                _swarmModemStatusCallback(status, data); // Call the callback
              }

              swarm_m138_free_char(data);
              return (true);
            }
          }
        }
      }
      swarm_m138_free_char(data);
    }
  }
  { // $SL - Sleep Mode
    Swarm_M138_Wake_Cause_e cause = SWARM_M138_WAKE_CAUSE_INVALID;
    char *eventStart;
    char *eventEnd;

    eventStart = strstr(event, "$SL WAKE,");
    if (eventStart != NULL)
    {
      eventEnd = strchr(eventStart, '*'); // Stop at the asterix
      if (eventEnd != NULL)
      {
        // Check for the wake cause
        if (strstr(eventStart, "WAKE,GPIO") != NULL)
          cause = SWARM_M138_WAKE_CAUSE_GPIO;
        else if (strstr(eventStart, "WAKE,SERIAL") != NULL)
          cause = SWARM_M138_WAKE_CAUSE_SERIAL;
        else if (strstr(eventStart, "WAKE,TIME") != NULL)
          cause = SWARM_M138_WAKE_CAUSE_TIME;

        if (cause < SWARM_M138_WAKE_CAUSE_INVALID)
        {
          if (_swarmSleepWakeCallback != NULL)
          {
            _swarmSleepWakeCallback(cause); // Call the callback
          }

          return (true);
        }
      }
    }
  }
  { // $RD - Receive Data Message
    char *eventStart;
    char *eventEnd;
    bool appIDseen = false;
    int appID_i, rssi_i = 0, snr_i = 0, fdev_i = 0;
    uint16_t appID = 0;
    int16_t rssi = 0, snr = 0, fdev = 0;
    char *paramPtr;
    int ret = 0;

    eventStart = strstr(event, "$RD ");
    if (eventStart != NULL)
    {
      eventEnd = strchr(eventStart, '*'); // Stop at the asterix
      if (eventEnd != NULL)
      {
        // Check for the appID (shows only with firmware version v1.1.0+)
        paramPtr = strstr(eventStart, "AI="); // Look for the AI=
        if (paramPtr != NULL)
        {
          ret = sscanf(paramPtr, "AI=%d,", &appID_i);
          if (ret == 1)
          {
            appID = (uint16_t)appID_i;
            appIDseen = true; // Flag that the appID has been seen and extracted correctly
          }
        }

        // Extract the rssi, snt and fdev
        paramPtr = strstr(eventStart, "RSSI=");
        if (paramPtr != NULL)
        {
          ret = sscanf(paramPtr, "RSSI=%d,SNR=%d,FDEV=%d,", &rssi_i, &snr_i, &fdev_i);

          if (ret == 3)
          {
            rssi = (int16_t)rssi_i;
            snr = (int16_t)snr_i;
            fdev = (int16_t)fdev_i;

            // Extract the data (ASCII Hex)
            paramPtr = strstr(paramPtr, "FDEV="); // Find the FDEV
            if (paramPtr != NULL)
            {
              paramPtr = strchr(paramPtr, ','); // Find the comma after the FDEV
              if (paramPtr != NULL)
              {
                paramPtr++; // Point to the first ASCII Hex character
                *eventEnd = 0; // Change the asterix into NULL

                if (_swarmReceiveMessageCallback != NULL)
                {
                  if (appIDseen)
                    _swarmReceiveMessageCallback((const uint16_t *)&appID, (const int16_t *)&rssi,
                                                 (const int16_t *)&snr, (const int16_t *)&fdev, (const char *)paramPtr); // Call the callback
                  else
                    _swarmReceiveMessageCallback(NULL, (const int16_t *)&rssi,
                                                 (const int16_t *)&snr, (const int16_t *)&fdev, (const char *)paramPtr); // Call the callback
                }

                *eventEnd = '*'; // Be nice. Restore the asterix

                return (true);
              }
            }
          }
        }
      }
    }
  }
  { // $TD - Transmit Data Message
    char *eventStart;
    char *eventEnd;
    int rssi_i = 0, snr_i = 0, fdev_i = 0;
    uint64_t msg_id = 0;
    int16_t rssi = 0, snr = 0, fdev = 0;
    char *paramPtr;
    int ret = 0;

    eventStart = strstr(event, "$TD SENT");
    if (eventStart != NULL)
    {
      eventEnd = strchr(eventStart, '*'); // Stop at the asterix
      if (eventEnd != NULL)
      {
        // Extract the rssi, snt and fdev
        paramPtr = strstr(eventStart, "RSSI=");
        if (paramPtr != NULL)
        {
          ret = sscanf(paramPtr, "RSSI=%d,SNR=%d,FDEV=%d,", &rssi_i, &snr_i, &fdev_i);

          if (ret == 3)
          {
            rssi = (int16_t)rssi_i;
            snr = (int16_t)snr_i;
            fdev = (int16_t)fdev_i;

            // Extract the 64-bit message ID
            paramPtr = strstr(paramPtr, "FDEV="); // Find the FDEV
            if (paramPtr != NULL)
            {
              paramPtr = strchr(paramPtr, ','); // Find the comma after the FDEV
              if (paramPtr != NULL)
              {
                paramPtr++; // Point to the first message ID character

                while (paramPtr < eventEnd) // Add each character to id
                {
                  msg_id *= 10;
                  msg_id += (uint64_t)((*paramPtr) - '0');
                  paramPtr++;
                }

                if (_swarmTransmitDataCallback != NULL)
                {
                  _swarmTransmitDataCallback((const int16_t *)&rssi, (const int16_t *)&snr,
                                             (const int16_t *)&fdev, (const uint64_t *)&msg_id); // Call the callback
                }

                return (true);
              }
            }
          }
        }
      }
    }
  }
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
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5); // Clear it
  sprintf(command, "%s*", SWARM_M138_COMMAND_CONFIGURATION); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$CS DI=0x", "$CS ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$CS DI=0x");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Copy the response to *settings
    responseStart += 4; // Start at the 'D'
    memcpy(settings, responseStart, responseEnd - responseStart);

    // Add a null-terminator
    settings[responseEnd - responseStart] = 0;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
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
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;
  uint32_t dev_ID = 0;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_CONFIGURATION) + 5); // Clear it
  sprintf(command, "%s*", SWARM_M138_COMMAND_CONFIGURATION); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$CS DI=0x", "$CS ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$CS DI=0x");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, ','); // Stop at the comma
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
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
      else if ((c >= 'A') && (c <= 'F'))
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

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the most recent $DT message
    @param  dateTime
            A pointer to a Swarm_M138_DateTimeData_t struct which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getDateTime(Swarm_M138_DateTimeData_t *dateTime)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_DATE_TIME_STAT) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_DATE_TIME_STAT) + 7); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_DATE_TIME_STAT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$DT ", "$DT ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$DT ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 20))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the Date, Time and flag
    int year, month, day, hour, minute, second;
    char valid;

    int ret = sscanf(responseStart, "$DT %4d%2d%2d%2d%2d%2d,%c*", &year, &month, &day, &hour, &minute, &second, &valid);

    if (ret < 7)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    dateTime->YYYY = (uint16_t)year;
    dateTime->DD = (uint8_t)month;
    dateTime->MM = (uint8_t)day;
    dateTime->hh = (uint8_t)hour;
    dateTime->mm = (uint8_t)minute;
    dateTime->ss = (uint8_t)second;
    dateTime->valid = valid == 'V' ? 1 : 0;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query the current $DT rate
    @param  rate
            A pointer to a uint32_t which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getDateTimeRate(uint32_t *rate)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_DATE_TIME_STAT) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_DATE_TIME_STAT) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_DATE_TIME_STAT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$DT ", "$DT ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$DT ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the rate
    char c;
    uint32_t theRate = 0;
    responseStart += 4; // Point at the first digit of the rate

    c = *responseStart; // Get the first digit of the rate
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the rate one digit at a time
      {
        theRate = theRate * 10;
        theRate += (uint32_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the rate
    }

    if (c == ',') // If we hit a comma then this is an unsolicited dateTime message, not the rate
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    else
      *rate = theRate;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}


/**************************************************************************/
/*!
    @brief  Set the rate of $DT Date/Time messages
    @param  rate
            The interval between messages
            0 == Disable. Max is 2147483647 (2^31 - 1)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_RATE if the rate is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setDateTimeRate(uint32_t rate)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check rate is within bounds
  if (rate > SWARM_M138_MAX_MESSAGE_RATE)
    return (SWARM_M138_ERROR_INVALID_RATE);

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_DATE_TIME_STAT) + 1 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_DATE_TIME_STAT) + 1 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_DATE_TIME_STAT, rate); // Copy the command, add the asterix
#else
  sprintf(command, "%s %lu*", SWARM_M138_COMMAND_DATE_TIME_STAT, rate); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$DT OK*", "$DT ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Read the modem firmware version using the $FV message
    @param  settings
            A pointer to where the settings will be stored
            It is recommended that the user allocates at least
            SWARM_M138_MEM_ALLOC_FV bytes to store the settings
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getFirmwareVersion(char *version)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_FIRMWARE_VER) + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_FIRMWARE_VER) + 5); // Clear it
  sprintf(command, "%s*", SWARM_M138_COMMAND_FIRMWARE_VER); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$FV ", "$FV ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$FV ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Copy the response to *settings
    responseStart += 4; // Start at the 'D'
    memcpy(version, responseStart, responseEnd - responseStart);

    // Add a null-terminator
    version[responseEnd - responseStart] = 0;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the most recent $GJ message
    @param  dateTime
            A pointer to a Swarm_M138_DateTimeData_t struct which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGpsJammingIndication(Swarm_M138_GPS_Jamming_Indication_t *jamming)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPS_JAMMING) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPS_JAMMING) + 7); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_GPS_JAMMING); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GJ ", "$GJ ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GJ ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 3))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the spoof_state and jamming_level
    int spoof_state, jamming_level;

    int ret = sscanf(responseStart, "$GJ %d,%d*", &spoof_state, &jamming_level);

    if (ret < 2)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    jamming->spoof_state = (uint8_t)spoof_state;
    jamming->jamming_level = (uint8_t)jamming_level;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query the current $GJ rate
    @param  rate
            A pointer to a uint32_t which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGpsJammingIndicationRate(uint32_t *rate)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPS_JAMMING) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPS_JAMMING) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_GPS_JAMMING); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GJ ", "$GJ ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GJ ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the rate
    char c;
    uint32_t theRate = 0;
    responseStart += 4; // Point at the first digit of the rate

    c = *responseStart; // Get the first digit of the rate
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the rate one digit at a time
      {
        theRate = theRate * 10;
        theRate += (uint32_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the rate
    }

    if (c == ',') // If we hit a comma then this is an unsolicited jamming message, not the rate
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    else
      *rate = theRate;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}


/**************************************************************************/
/*!
    @brief  Set the rate of $GJ jamming indication messages
    @param  rate
            The interval between messages
            0 == Disable. Max is 2147483647 (2^31 - 1)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_RATE if the rate is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setGpsJammingIndicationRate(uint32_t rate)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check rate is within bounds
  if (rate > SWARM_M138_MAX_MESSAGE_RATE)
    return (SWARM_M138_ERROR_INVALID_RATE);

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPS_JAMMING) + 1 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPS_JAMMING) + 1 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_GPS_JAMMING, rate); // Copy the command, add the asterix
#else
  sprintf(command, "%s %lu*", SWARM_M138_COMMAND_GPS_JAMMING, rate); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GJ OK*", "$GJ ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the most recent $GN message
    @param  info
            A pointer to a Swarm_M138_GeospatialData_t struct which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGeospatialInfo(Swarm_M138_GeospatialData_t *info)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GEOSPATIAL_INFO) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GEOSPATIAL_INFO) + 7); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_GEOSPATIAL_INFO); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GN ", "$GN ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GN ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 10))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the geospatial info
    int latH, lonH, alt, course, speed;
    char latL[8], lonL[8];

    int ret = sscanf(responseStart, "$GN %d.%[^,],%d.%[^,],%d,%d,%d*",
                     &latH, latL, &lonH, lonL, &alt, &course, &speed);

    if (ret < 7)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    if (latH >= 0)
      info->lat = (float)latH + ((float)atol(latL) / pow(10, strlen(latL)));
    else
      info->lat = (float)latH - ((float)atol(latL) / pow(10, strlen(latL)));
    if (lonH >= 0)
      info->lon = (float)lonH + ((float)atol(lonL) / pow(10, strlen(lonL)));
    else
      info->lon = (float)lonH - ((float)atol(lonL) / pow(10, strlen(lonL)));
    info->alt = (float)alt;
    info->course = (float)course;
    info->speed = (float)speed;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query the current $GN rate
    @param  rate
            A pointer to a uint32_t which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGeospatialInfoRate(uint32_t *rate)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GEOSPATIAL_INFO) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GEOSPATIAL_INFO) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_GEOSPATIAL_INFO); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GN ", "$GN ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GN ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the rate
    char c;
    uint32_t theRate = 0;
    responseStart += 4; // Point at the first digit of the rate

    c = *responseStart; // Get the first digit of the rate
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the rate one digit at a time
      {
        theRate = theRate * 10;
        theRate += (uint32_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the rate
    }

    if (c == ',') // If we hit a comma then this is an unsolicited geo message, not the rate
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    else
      *rate = theRate;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}


/**************************************************************************/
/*!
    @brief  Set the rate of $GN geospatial information messages
    @param  rate
            The interval between messages
            0 == Disable. Max is 2147483647 (2^31 - 1)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_RATE if the rate is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setGeospatialInfoRate(uint32_t rate)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check rate is within bounds
  if (rate > SWARM_M138_MAX_MESSAGE_RATE)
    return (SWARM_M138_ERROR_INVALID_RATE);

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GEOSPATIAL_INFO) + 1 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GEOSPATIAL_INFO) + 1 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_GEOSPATIAL_INFO, rate); // Copy the command, add the asterix
#else
  sprintf(command, "%s %lu*", SWARM_M138_COMMAND_GEOSPATIAL_INFO, rate); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GN OK*", "$GN ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the current GPIO1 pin mode using the $GP message
    @param  mode
            A pointer to a Swarm_M138_GPIO1_Mode_e enum where the mode will be stored
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGPIO1Mode(Swarm_M138_GPIO1_Mode_e *mode)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPIO1_CONTROL) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPIO1_CONTROL) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_GPIO1_CONTROL); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GP ", "$GP ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GP ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the mode
    int theMode;
    int ret = sscanf(responseStart, "$GP %d*", &theMode);

    if (ret < 1)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    *mode = (Swarm_M138_GPIO1_Mode_e)theMode; // Copy the extracted mode into mode
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the current GPIO1 pin mode using the $GP message
    @param  mode
            A pointer to a Swarm_M138_GPIO1_Mode_e enum where the mode will be stored
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_MODE if the pin mode is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setGPIO1Mode(Swarm_M138_GPIO1_Mode_e mode)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check mode is within bounds
  if ((int)mode >= SWARM_M138_GPIO1_INVALID)
    return (SWARM_M138_ERROR_INVALID_MODE);

  // Allocate memory for the command, mode, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPIO1_CONTROL) + 1 + 2 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPIO1_CONTROL) + 1 + 2 + 5); // Clear it
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_GPIO1_CONTROL, (int)mode); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GP OK*", "$GP ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Read the voltage on GPIO1 (Modes 1 and 2 only)
    @param  voltage
            A pointer to a float where the voltage will be stored.
            Mode 1 (ADC) returns a true voltage.
            Mode 2 (INPUT) returns 0.00 for low or 3.30 for high.
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_MODE if the pin mode is invalid ("$GP Cannot read in mode")
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::readGPIO1voltage(float *voltage)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, space, @, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPIO1_CONTROL) + 1 + 1 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPIO1_CONTROL) + 1 + 1 + 5); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_GPIO1_CONTROL); // Copy the command, add the @ and asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GP ", "$GP ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GP ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 5))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Parse the H/L (mode 2) or the voltage (mode 1)
    float volts = -10.00;

    char *highLow = strstr(responseStart, "$GP L*");

    if (highLow != NULL)
    {
      volts = 0.00;
    }
    else
    {
      highLow = strstr(responseStart, "$GP H*");

      if (highLow != NULL)
      {
        volts = 3.30;
      }
      else
      {
        int volt;
        char mVolt[5];

        int ret = sscanf(responseStart, "$GP %d.%[^,]V*", &volt, mVolt);

        if (ret == 2)
        {
          volts = (float)volt;
          volts += (float)atol(mVolt) / pow(10, strlen(mVolt)); // Assumes voltage is always +ve!
        }
      }
    }

    if (volts >= 0.0)
      *voltage = volts;
    else
      err = SWARM_M138_ERROR_INVALID_MODE;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the most recent $GS message
    @param  fixQuality
            A pointer to a Swarm_M138_GPS_Fix_Quality_t struct which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGpsFixQuality(Swarm_M138_GPS_Fix_Quality_t *fixQuality)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPS_FIX_QUAL) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPS_FIX_QUAL) + 7); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_GPS_FIX_QUAL); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GS ", "$GS ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GS ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 11))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the GPS fix quality
    int hdop, vdop, gnss_sats, unused;
    char fix_type[3];

    int ret = sscanf(responseStart, "$GS %d,%d,%d,%d,%c%c*", &hdop, &vdop, &gnss_sats, &unused, &fix_type[0], &fix_type[1]);

    if (ret < 6)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    fixQuality->hdop = (uint16_t)hdop;
    fixQuality->vdop = (uint16_t)vdop;
    fixQuality->gnss_sats = (uint8_t)gnss_sats;
    fixQuality->unused = (uint8_t)unused;

    fix_type[2] = 0; // Null-terminate the fix type
    if (strstr(fix_type, "NF") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_NF;
    else if (strstr(fix_type, "DR") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_DR;
    else if (strstr(fix_type, "G2") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_G2;
    else if (strstr(fix_type, "G3") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_G3;
    else if (strstr(fix_type, "D2") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_D2;
    else if (strstr(fix_type, "D3") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_D3;
    else if (strstr(fix_type, "RK") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_RK;
    else if (strstr(fix_type, "TT") != NULL)
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_TT;
    else
      fixQuality->fix_type = SWARM_M138_GPS_FIX_TYPE_INVALID;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query the current $GS rate
    @param  rate
            A pointer to a uint32_t which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getGpsFixQualityRate(uint32_t *rate)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPS_FIX_QUAL) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPS_FIX_QUAL) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_GPS_FIX_QUAL); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GS ", "$GS ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$GS ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the rate
    char c;
    uint32_t theRate = 0;
    responseStart += 4; // Point at the first digit of the rate

    c = *responseStart; // Get the first digit of the rate
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the rate one digit at a time
      {
        theRate = theRate * 10;
        theRate += (uint32_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the rate
    }

    if (c == ',') // If we hit a comma then this is an unsolicited fix message, not the rate
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    else
      *rate = theRate;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}


/**************************************************************************/
/*!
    @brief  Set the rate of $GS GPS fix quality messages
    @param  rate
            The interval between messages
            0 == Disable. Max is 2147483647 (2^31 - 1)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_RATE if the rate is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setGpsFixQualityRate(uint32_t rate)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check rate is within bounds
  if (rate > SWARM_M138_MAX_MESSAGE_RATE)
    return (SWARM_M138_ERROR_INVALID_RATE);

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_GPS_FIX_QUAL) + 1 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_GPS_FIX_QUAL) + 1 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_GPS_FIX_QUAL, rate); // Copy the command, add the asterix
#else
  sprintf(command, "%s %lu*", SWARM_M138_COMMAND_GPS_FIX_QUAL, rate); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$GS OK*", "$GS ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  The Modem enters a low power mode until power is completely removed and restored
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::powerOff(void)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_POWER_OFF) + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_POWER_OFF) + 5); // Clear it
  sprintf(command, "%s*", SWARM_M138_COMMAND_POWER_OFF); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$PO OK*", "$PO ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the most recent $PW message
    @param  powerStatus
            A pointer to a Swarm_M138_Power_Status_t struct which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getPowerStatus(Swarm_M138_Power_Status_t *powerStatus)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_POWER_STAT) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_POWER_STAT) + 7); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_POWER_STAT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$PW ", "$PW ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$PW ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 10))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the power status
    int unused1H, unused2H, unused3H, cpu_voltsH, tempH;
    char unused1L[8], unused2L[8], unused3L[8], cpu_voltsL[8], tempL[8];

    int ret = sscanf(responseStart, "$PW %d.%[^,],%d.%[^,],%d.%[^,],%d.%[^,],%d.%[^,]*",
                     &cpu_voltsH, cpu_voltsL, &unused1H, unused1L,
                     &unused2H, unused2L, &unused3H, unused3L,
                     &tempH, tempL);

    if (ret < 10)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    if (cpu_voltsH >= 0)
      powerStatus->cpu_volts = (float)cpu_voltsH + ((float)atol(cpu_voltsL) / pow(10, strlen(cpu_voltsL)));
    else
      powerStatus->cpu_volts = (float)cpu_voltsH - ((float)atol(cpu_voltsL) / pow(10, strlen(cpu_voltsL)));
    if (unused1H >= 0)
      powerStatus->unused1 = (float)unused1H + ((float)atol(unused1L) / pow(10, strlen(unused1L)));
    else
      powerStatus->unused1 = (float)unused1H - ((float)atol(unused1L) / pow(10, strlen(unused1L)));
    if (unused2H >= 0)
      powerStatus->unused2 = (float)unused2H + ((float)atol(unused2L) / pow(10, strlen(unused2L)));
    else
      powerStatus->unused2 = (float)unused2H - ((float)atol(unused2L) / pow(10, strlen(unused2L)));
    if (unused3H >= 0)
      powerStatus->unused3 = (float)unused3H + ((float)atol(unused3L) / pow(10, strlen(unused3L)));
    else
      powerStatus->unused3 = (float)unused3H - ((float)atol(unused3L) / pow(10, strlen(unused3L)));
    if (tempH >= 0)
      powerStatus->temp = (float)tempH + ((float)atol(tempL) / pow(10, strlen(tempL)));
    else
      powerStatus->temp = (float)tempH - ((float)atol(tempL) / pow(10, strlen(tempL)));
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query the current $PW rate
    @param  rate
            A pointer to a uint32_t which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getPowerStatusRate(uint32_t *rate)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_POWER_STAT) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_POWER_STAT) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_POWER_STAT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$PW ", "$PW ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$PW ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the rate
    char c;
    uint32_t theRate = 0;
    responseStart += 4; // Point at the first digit of the rate

    c = *responseStart; // Get the first digit of the rate
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the rate one digit at a time
      {
        theRate = theRate * 10;
        theRate += (uint32_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the rate
    }

    if (c == ',') // If we hit a comma then this is an unsolicited power message, not the rate
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    else
      *rate = theRate;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}


/**************************************************************************/
/*!
    @brief  Set the rate of $PW power status messages
    @param  rate
            The interval between messages
            0 == Disable. Max is 2147483647 (2^31 - 1)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_RATE if the rate is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setPowerStatusRate(uint32_t rate)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check rate is within bounds
  if (rate > SWARM_M138_MAX_MESSAGE_RATE)
    return (SWARM_M138_ERROR_INVALID_RATE);

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_POWER_STAT) + 1 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_POWER_STAT) + 1 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_POWER_STAT, rate); // Copy the command, add the asterix
#else
  sprintf(command, "%s %lu*", SWARM_M138_COMMAND_POWER_STAT, rate); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$PW OK*", "$PW ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the modem temperature
    @param  temperature
            A pointer to a float which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getTemperature(float *temperature)
{
  Swarm_M138_Power_Status_t *powerStatus = new Swarm_M138_Power_Status_t;
  Swarm_M138_Error_e err = getPowerStatus(powerStatus);
  if (err == SWARM_M138_ERROR_SUCCESS)
    *temperature = powerStatus->temp;
  delete powerStatus;
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the modem CPU voltage
    @param  voltage
            A pointer to a float which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getCPUvoltage(float *voltage)
{
  Swarm_M138_Power_Status_t *powerStatus = new Swarm_M138_Power_Status_t;
  Swarm_M138_Error_e err = getPowerStatus(powerStatus);
  if (err == SWARM_M138_ERROR_SUCCESS)
    *voltage = powerStatus->cpu_volts;
  delete powerStatus;
  return (err);
}

/**************************************************************************/
/*!
    @brief  Restart the modem
    @param  deletedb
            Bool: If true, the database will be cleared. Default is false.
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::restartDevice(bool deletedb)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  size_t msgLen = strlen(SWARM_M138_COMMAND_RESTART) + 5;
  if (deletedb) msgLen += 9; // space deletedb
  command = swarm_m138_alloc_char(msgLen);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, msgLen); // Clear it
  if (deletedb)
    sprintf(command, "%s deletedb*", SWARM_M138_COMMAND_RESTART); // Copy the command, add the asterix
  else
    sprintf(command, "%s*", SWARM_M138_COMMAND_RESTART); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$RS OK*", "$RS ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Get the most recent $RT message
    @param  rxTest
            A pointer to a Swarm_M138_Receive_Test_t struct which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getReceiveTest(Swarm_M138_Receive_Test_t *rxTest)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_RX_TEST) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_RX_TEST) + 7); // Clear it
  sprintf(command, "%s @*", SWARM_M138_COMMAND_RX_TEST); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$RT ", "$RT ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$RT ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL) || (responseEnd < (responseStart + 9))) // Check we have enough data
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the receive test info
    int rssi_bg = 0, rssi_sat = 0, snr = 0, fdev = 0;
    int YYYY = 0, MM = 0, DD = 0, hh = 0, mm = 0, ss = 0;
    uint32_t sat_ID = 0;

    int ret = sscanf(responseStart, "$RT RSSI=%d,SNR=%d,FDEV=%d,TS=%d-%d-%dT%d:%d:%d,DI=0x",
                     &rssi_sat, &snr, &fdev, &YYYY, &MM, &DD, &hh, &mm, &ss);

    if (ret == 9)
    {
      responseStart = strstr(response, "DI=0x"); // Find the start of the satellite ID

      // Extract the ID
      responseStart += 5; // Point at the first digit
      while (responseStart < responseEnd)
      {
        sat_ID <<= 4; // Shuffle the existing value along by 4 bits
        char c = *responseStart; // Get the digit
        if ((c >= '0') && (c <= '9'))
          sat_ID |= c - '0';
        else if ((c >= 'a') && (c <= 'f'))
          sat_ID |= c + 10 - 'a';
        else if ((c >= 'A') && (c <= 'F'))
          sat_ID |= c + 10 - 'A';
        responseStart++;
      }
    }
    else // Try to extract just rssi_background
    {
      ret = sscanf(responseStart, "$RT RSSI=%d*", &rssi_bg);
    }

    if ((ret == 9) || (ret == 1)) // Check if we got valid data
    {
      rxTest->background = ret == 1;
      rxTest->rssi_background = (int16_t)rssi_bg;
      rxTest->rssi_sat = (int16_t)rssi_sat;
      rxTest->snr = (int16_t)snr;
      rxTest->fdev = (int16_t)fdev;
      rxTest->time.YYYY = YYYY;
      rxTest->time.MM = MM;
      rxTest->time.DD = DD;
      rxTest->time.hh = hh;
      rxTest->time.mm = mm;
      rxTest->time.ss = ss;
      rxTest->sat_id = sat_ID;
    }
    else
      err = SWARM_M138_ERROR_ERROR;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query the current $RT rate
    @param  rate
            A pointer to a uint32_t which will hold the result
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getReceiveTestRate(uint32_t *rate)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_RX_TEST) + 7);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_RX_TEST) + 7); // Clear it
  sprintf(command, "%s ?*", SWARM_M138_COMMAND_RX_TEST); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$RT ", "$RT ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$RT ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the rate
    char c;
    uint32_t theRate = 0;
    responseStart += 4; // Point at the first digit of the rate

    c = *responseStart; // Get the first digit of the rate
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the rate one digit at a time
      {
        theRate = theRate * 10;
        theRate += (uint32_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the rate
    }

    if (c == ',') // If we hit a comma then this is an unsolicited RT message, not the rate
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    else
      *rate = theRate;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}


/**************************************************************************/
/*!
    @brief  Set the rate of $RT receive test messages
    @param  rate
            The interval between messages
            0 == Disable. Max is 2147483647 (2^31 - 1)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_INVALID_RATE if the rate is invalid
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setReceiveTestRate(uint32_t rate)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Check rate is within bounds
  if (rate > SWARM_M138_MAX_MESSAGE_RATE)
    return (SWARM_M138_ERROR_INVALID_RATE);

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_RX_TEST) + 1 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_RX_TEST) + 1 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s %u*", SWARM_M138_COMMAND_RX_TEST, rate); // Copy the command, add the asterix
#else
  sprintf(command, "%s %lu*", SWARM_M138_COMMAND_RX_TEST, rate); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$RT OK*", "$RT ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Instruct the modem to sleep for this many seconds
    @param  seconds
            The sleep duration in seconds
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::sleepMode(uint32_t seconds)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_SLEEP) + 3 + 10 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_SLEEP) + 3 + 10 + 5); // Clear it
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s S=%d*", SWARM_M138_COMMAND_SLEEP, seconds); // Copy the command, add the asterix
#else
  sprintf(command, "%s S=%ld*", SWARM_M138_COMMAND_SLEEP, seconds); // Copy the command, add the asterix
#endif
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$SL OK*", "$SL ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Instruct the modem to sleep until this date and time
    @param  sleepUntil
            A Swarm_M138_DateTimeData_t struct containing the date and time the modem should sleep until
    @param  dateAndTime
            If true (default), the modem will sleep until the specified date and time
            If false, the modem will sleep until the specified time
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::sleepMode(Swarm_M138_DateTimeData_t sleepUntil, bool dateAndTime)
{
  char *command;
  char *response;
  char *scratchpad;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, rate, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_SLEEP) + 3 + 19 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_SLEEP) + 3 + 19 + 5); // Clear it

  scratchpad = swarm_m138_alloc_char(16); // Create a scratchpad to hold the date/time
  if (scratchpad == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(scratchpad, 0, 16); // Clear it

  sprintf(command, "%s U=", SWARM_M138_COMMAND_SLEEP); // Copy the command

  // Note: some platforms do not support sprintf "%02d" correctly. We need to add the preceding zero manually...

  if (dateAndTime) // Check if we need to include the date
  {
    sprintf(scratchpad, "%d", sleepUntil.YYYY); // Add the year
    strcat(command, scratchpad);
    strcat(command, "-");
    sprintf(scratchpad, "%d", sleepUntil.MM); // Add the month
    if (sleepUntil.MM < 10) strcat(command, "0");
    strcat(command, scratchpad);
    strcat(command, "-");
    sprintf(scratchpad, "%d", sleepUntil.DD); // Add the day of month
    if (sleepUntil.DD < 10) strcat(command, "0");
    strcat(command, scratchpad);
    strcat(command, "T");
  }

  sprintf(scratchpad, "%d", sleepUntil.hh); // Add the hour
  if (sleepUntil.hh < 10) strcat(command, "0");
  strcat(command, scratchpad);
  strcat(command, ":");
  sprintf(scratchpad, "%d", sleepUntil.mm); // Add the minute
  if (sleepUntil.mm < 10) strcat(command, "0");
  strcat(command, scratchpad);
  strcat(command, ":");
  sprintf(scratchpad, "%d", sleepUntil.ss); // Add the second
  if (sleepUntil.ss < 10) strcat(command, "0");
  strcat(command, scratchpad);

  strcat(command, "*"); // Add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(scratchpad);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$SL OK*", "$SL ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(scratchpad);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Return the count of all messages (default) or unread messages (unread = true)
    @param  count
            A pointer to a uint16_t which will hold the message count
    @param  unread
            If false (default): returns the count of all messages
            If true: returns the count of the unread messages
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getRxMessageCount(uint16_t *count, bool unread)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9); // Clear it
  if (unread)
    sprintf(command, "%s C=U*", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  else
    sprintf(command, "%s C=**", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MM ", "$MM ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_READ_TIMEOUT);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$MM ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the count
    char c;
    uint16_t theCount = 0;
    responseStart += 4; // Point at the first digit of the count

    c = *responseStart; // Get the first digit of the count
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix or a comma
    {
      if ((c >= '0') && (c <= '9')) // Extract the count one digit at a time
      {
        theCount = theCount * 10;
        theCount += (uint16_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the count
    }

    if (c == ',') // If we hit a comma, this must be a different $MM message
    {
      *count = 0; // Set count to zero in case the user is not checking err
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    }
    else
    {
      *count = theCount;
    }
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Delete the RX message with the specified ID
    @param  msg_id
            The ID of the message to be deleted
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::deleteRxMessage(uint64_t msg_id)
{
  char *command;
  char *response;
  char *fwd;
  char *rev;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 3 + 20 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 3 + 20 + 5); // Clear it

  // Allocate memory for the scratchpads
  fwd = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (fwd == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(fwd, 0, 21); // Clear it
  rev = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (rev == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(rev, 0, 21); // Clear it

  sprintf(command, "%s D=", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command

  // Add the 64-bit message ID
  // Based on printLLNumber by robtillaart
  // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
  unsigned int i = 0;
  if (msg_id == 0ULL) // if msg_id is zero, set fwd to "0"
  {
    fwd[0] = '0';
  }
  else
  {
    while (msg_id > 0)
    {
      rev[i++] = (msg_id % 10) + '0'; // divide by 10, convert the remainder to char
      msg_id /= 10; // divide by 10
    }
    unsigned int j = 0;
    while (i > 0)
    {
      fwd[j++] = rev[--i]; // reverse the order
    }
  }
  strcat(command, fwd);

  strcat(command, "*"); // Append the asterix

  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    swarm_m138_free_char(rev);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MM DELETED", "$MM ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_DELETE_TIMEOUT);

  swarm_m138_free_char(command);
  swarm_m138_free_char(fwd);
  swarm_m138_free_char(rev);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Delete all read RX messages (default) or all messages (read = false)
    @param  read
            If true (default): deletes the read messages
            If false: deletes all messages
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::deleteAllRxMessages(bool read)
{
  char *command;
  char *response;
  char *scratchpad;
  Swarm_M138_Error_e err;
  uint16_t msgTotal = 0, unreadTotal = 0;

  err = getRxMessageCount(&msgTotal, false); // Get the message count so we know how many messages to expect
  if (err != SWARM_M138_ERROR_SUCCESS)
    return (err);

  err = getRxMessageCount(&unreadTotal, true); // Get the unread message count so we know how many messages to expect
  if (err != SWARM_M138_ERROR_SUCCESS)
    return (err);

  if (unreadTotal > msgTotal) // Sanity check
    return (SWARM_M138_ERROR_ERROR);

  // If we are deleting all messages then msgTotal is correct
  // If we are deleting only the read messages, then we need to subtract unreadTotal from msgTotal
  if (read)
    msgTotal -= unreadTotal;

  if (_printDebug == true)
  {
    _debugPort->print(F("deleteAllRxMessages: msgTotal is "));
    _debugPort->println(msgTotal);
  }

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9); // Clear it
  if (read)
    sprintf(command, "%s D=R*", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  else
    sprintf(command, "%s D=**", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  scratchpad = swarm_m138_alloc_char(16); // Create a scratchpad to hold the expectedResponse
  if (scratchpad == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(response);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(scratchpad, 0, 16); // Clear it

  sprintf(scratchpad, "$MM %d*", msgTotal); // Create the expected response

  err = sendCommandWithResponse(command, scratchpad, "$MM ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_DELETE_TIMEOUT);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  swarm_m138_free_char(scratchpad);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Mark the RX message with the specified ID as read
    @param  msg_id
            The ID of the message to be marked as read
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::markRxMessage(uint64_t msg_id)
{
  char *command;
  char *response;
  char *fwd;
  char *rev;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 3 + 20 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 3 + 20 + 5); // Clear it

  // Allocate memory for the scratchpads
  fwd = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (fwd == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(fwd, 0, 21); // Clear it
  rev = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (rev == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(rev, 0, 21); // Clear it

  sprintf(command, "%s M=", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command

  // Add the 64-bit message ID
  // Based on printLLNumber by robtillaart
  // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
  unsigned int i = 0;
  if (msg_id == 0ULL) // if msg_id is zero, set fwd to "0"
  {
    fwd[0] = '0';
  }
  else
  {
    while (msg_id > 0)
    {
      rev[i++] = (msg_id % 10) + '0'; // divide by 10, convert the remainder to char
      msg_id /= 10; // divide by 10
    }
    unsigned int j = 0;
    while (i > 0)
    {
      fwd[j++] = rev[--i]; // reverse the order
    }
  }
  strcat(command, fwd);

  strcat(command, "*"); // Append the asterix

  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    swarm_m138_free_char(rev);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MM MARKED", "$MM ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_READ_TIMEOUT);

  swarm_m138_free_char(command);
  swarm_m138_free_char(fwd);
  swarm_m138_free_char(rev);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Mark all RX messages as read
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::markAllRxMessages(void)
{
  char *command;
  char *response;
  char *scratchpad;
  Swarm_M138_Error_e err;
  uint16_t msgTotal = 0;

  err = getRxMessageCount(&msgTotal, false); // Get the message count so we know how many messages to expect
  if (err != SWARM_M138_ERROR_SUCCESS)
    return (err);

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9); // Clear it
  sprintf(command, "%s M=**", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  scratchpad = swarm_m138_alloc_char(16); // Create a scratchpad to hold the expectedResponse
  if (scratchpad == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(response);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(scratchpad, 0, 16); // Clear it

  sprintf(scratchpad, "$MM %d*", msgTotal); // Create the expected response

  err = sendCommandWithResponse(command, scratchpad, "$MM ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_READ_TIMEOUT);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  swarm_m138_free_char(scratchpad);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Query if message notifications are enabled
    @param  enabled
            A pointer to a bool to hold the status
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getMessageNotifications(bool *enabled)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9); // Clear it
  sprintf(command, "%s N=?*", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MM N=", "$MM ERR", response, _RxBuffSize);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    char *enabledPtr = strstr(response, "$MM N=");
    if (enabledPtr != NULL)
      *enabled = *(enabledPtr + 6) == 'E';
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Enable / disable receive message notifications
    @param  enable
            If true: enable $RD message notifications
            If false: disable $RD message notifications
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::setMessageNotifications(bool enable)
{
  char *command;
  char *response;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 9); // Clear it
  if (enable)
    sprintf(command, "%s N=E*", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  else
    sprintf(command, "%s N=D*", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MM OK*", "$MM ERR", response, _RxBuffSize);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  List the message with the specified ID. Does not change message state
    @param  msg_id
            The ID of the message to be listed
    @param  asciiHex
            A pointer to a char array to hold the message
    @param  len
            The maximum message length which asciiHex can hold
    @param  epoch
            Optional: a pointer to a uint32_t to hold the epoch at which the modem received the message
    @param  appID
            Optional: a pointer to a uint16_t to hold the message appID if there is one
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::listMessage(uint64_t msg_id, char *asciiHex, size_t len, uint32_t *epoch, uint16_t *appID)
{
  return (readMessageInternal('L', msg_id, asciiHex, len, NULL, epoch, appID));
}
/**************************************************************************/
/*!
    @brief  Read the message with the specified ID
    @param  msg_id
            The ID of the message to be read
    @param  asciiHex
            A pointer to a char array to hold the message
    @param  len
            The maximum message length which asciiHex can hold
    @param  epoch
            Optional: a pointer to a uint32_t to hold the epoch at which the modem received the message
    @param  appID
            Optional: a pointer to a uint16_t to hold the message appID if there is one
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::readMessage(uint64_t msg_id, char *asciiHex, size_t len, uint32_t *epoch, uint16_t *appID)
{
  return (readMessageInternal('R', msg_id, asciiHex, len, NULL, epoch, appID));
}
/**************************************************************************/
/*!
    @brief  Read the oldest unread message
    @param  asciiHex
            A pointer to a char array to hold the message
    @param  len
            The maximum message length which asciiHex can hold
    @param  msg_id
            A pointer to a uint64_t to hold the message ID
    @param  epoch
            Optional: a pointer to a uint32_t to hold the epoch at which the modem received the message
    @param  appID
            Optional: a pointer to a uint16_t to hold the message appID if there is one
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::readOldestMessage(char *asciiHex, size_t len, uint64_t *msg_id, uint32_t *epoch, uint16_t *appID)
{
  return (readMessageInternal('O', 0, asciiHex, len, msg_id, epoch, appID));
}
/**************************************************************************/
/*!
    @brief  Read the newest unread message
    @param  asciiHex
            A pointer to a char array to hold the message
    @param  len
            The maximum message length which asciiHex can hold
    @param  msg_id
            A pointer to a uint64_t to hold the message ID
    @param  epoch
            Optional: a pointer to a uint32_t to hold the epoch at which the modem received the message
    @param  appID
            Optional: a pointer to a uint16_t to hold the message appID if there is one
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::readNewestMessage(char *asciiHex, size_t len, uint64_t *msg_id, uint32_t *epoch, uint16_t *appID)
{
  return (readMessageInternal('N', 0, asciiHex, len, msg_id, epoch, appID));
}

Swarm_M138_Error_e SWARM_M138::readMessageInternal(const char mode, uint64_t msg_id_in, char *asciiHex, size_t len, uint64_t *msg_id_out, uint32_t *epoch, uint16_t *appID)
{
  char *command;
  char *response;
  char *fwd;
  char *rev;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  memset(asciiHex, 0, len); // Clear the char array

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 3 + 20 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_RX_MGMT) + 3 + 20 + 5); // Clear it

  if (mode == 'L')
    sprintf(command, "%s L=", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command
  else
    sprintf(command, "%s R=", SWARM_M138_COMMAND_MSG_RX_MGMT); // Copy the command

  if ((mode == 'L') || (mode == 'R')) // L=msgID or R=msgID
  {
    // Allocate memory for the scratchpads
    fwd = swarm_m138_alloc_char(21); // Up to 20 digits plus null
    if (fwd == NULL)
    {
      swarm_m138_free_char(command);
      return (SWARM_M138_ERROR_MEM_ALLOC);
    }
    memset(fwd, 0, 21); // Clear it
    rev = swarm_m138_alloc_char(21); // Up to 20 digits plus null
    if (rev == NULL)
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(fwd);
      return (SWARM_M138_ERROR_MEM_ALLOC);
    }
    memset(rev, 0, 21); // Clear it

    // Add the 64-bit message ID
    // Based on printLLNumber by robtillaart
    // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
    unsigned int i = 0;
    if (msg_id_in == 0ULL) // if msg_id is zero, set fwd to "0"
    {
      fwd[0] = '0';
    }
    else
    {
      while (msg_id_in > 0)
      {
        rev[i++] = (msg_id_in % 10) + '0'; // divide by 10, convert the remainder to char
        msg_id_in /= 10; // divide by 10
      }
      unsigned int j = 0;
      while (i > 0)
      {
        fwd[j++] = rev[--i]; // reverse the order
      }
    }
    strcat(command, fwd);

    swarm_m138_free_char(fwd);
    swarm_m138_free_char(rev);
  }
  else if (mode == 'O') // R=O (Oldest)
    strcat(command, "O");
  else // if (mode == 'N') // R=N (Newest)
    strcat(command, "N");

  strcat(command, "*"); // Append the asterix

  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MM AI=", "$MM ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_READ_TIMEOUT);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$MM AI="); // Find the start of the response
    if (responseStart != NULL)
    {
      responseEnd = strchr(responseStart, '*'); // Find the asterix
      if (responseEnd != NULL)
      {
        // Extract the appID if required
        if (appID != NULL)
        {
          int appID_i = 0;
          int ret = sscanf(responseStart, "$MM AI=%d,", &appID_i);
          if (ret == 1)
          {
            *appID = (uint16_t)appID_i;
          }
        }

        responseStart = strchr(responseStart, ','); // Find the first comma (we know it is there)
        
        if (responseStart != NULL)
        {
          responseStart++; // Point to the first digit of the ASCII Hex
          size_t charsRead = 0;
          char c = *responseStart;
          while ((c != ',') && (responseStart < responseEnd) && (charsRead < len)) // Stop at the next comma
          {
            asciiHex[charsRead] = c; // Copy the char into asciiHex
            responseStart++; // Increment the pointer
            charsRead++; // Increment the counter
            c = *responseStart; // Get the next char
          }

          if (msg_id_out != NULL) // Are we reading the oldest or newest message?
          {
            responseStart++; // Point to the first digit of the msg_id
            uint64_t theID = 0;
            char c = *responseStart;
            while ((c != ',') && (responseStart < responseEnd))
            {
              // if (_printDebug == true)
              //   _debugPort->write(c);
              theID *= 10;
              theID += (uint64_t)(c - '0');
              responseStart++;
              c = *responseStart;
            }
            *msg_id_out = theID; // Store the extracted ID
          }
          else
          {
            responseStart++; // Point to the first digit of the msg_id
            responseStart = strchr(responseStart, ','); // Find the next comma
          }

          if (epoch != NULL) // Check if epoch is NULL
          {
            if (responseStart != NULL)
            {
              responseStart++; // Point to the first digit of the epoch
              uint32_t theEpoch = 0;
              char c = *responseStart;
              while ((c != '*') && (responseStart < responseEnd)) // Stop at the asterix
              {
                theEpoch *= 10;
                theEpoch += (uint32_t)(c - '0');
                responseStart++;
                c = *responseStart;
              }

              *epoch = theEpoch;
            }
          }
        }
        else
          err = SWARM_M138_ERROR_ERROR;
      }
      else
        err = SWARM_M138_ERROR_ERROR;
    }
    else
      err = SWARM_M138_ERROR_ERROR;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Return the count of all unsent messages
    @param  count
            A pointer to a uint16_t which will hold the message count
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::getUnsentMessageCount(uint16_t *count)
{
  char *command;
  char *response;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 9); // Clear it
  sprintf(command, "%s C=U*", SWARM_M138_COMMAND_MSG_TX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MT ", "$MT ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_READ_TIMEOUT);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$MT ");
    if (responseStart != NULL)
      responseEnd = strchr(responseStart, '*'); // Stop at the asterix
    if ((responseStart == NULL) || (responseEnd == NULL))
    {
      swarm_m138_free_char(command);
      swarm_m138_free_char(response);
      return (SWARM_M138_ERROR_ERROR);
    }

    // Extract the count
    char c;
    uint16_t theCount = 0;
    responseStart += 4; // Point at the first digit of the count

    c = *responseStart; // Get the first digit of the count
    while ((c != '*') && (c != ',')) // Keep going until we hit the asterix
    {
      if ((c >= '0') && (c <= '9')) // Extract the count one digit at a time
      {
        theCount = theCount * 10;
        theCount += (uint16_t)(c - '0');
      }
      responseStart++;
      c = *responseStart; // Get the next digit of the count
    }

    if (c == ',') // If we hit a comma, this must be a different $MT message
    {
      *count = 0; // Set count to zero in case the user is not checking err
      err = SWARM_M138_ERROR_INVALID_FORMAT;
    }
    else
    {
      *count = theCount;
    }
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Delete the TX message with the specified ID
    @param  msg_id
            The ID of the message to be deleted
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::deleteTxMessage(uint64_t msg_id)
{
  char *command;
  char *response;
  char *fwd;
  char *rev;
  Swarm_M138_Error_e err;

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 3 + 20 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 3 + 20 + 5); // Clear it

  // Allocate memory for the scratchpads
  fwd = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (fwd == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(fwd, 0, 21); // Clear it
  rev = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (rev == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(rev, 0, 21); // Clear it

  sprintf(command, "%s D=", SWARM_M138_COMMAND_MSG_TX_MGMT); // Copy the command

  // Add the 64-bit message ID
  // Based on printLLNumber by robtillaart
  // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
  unsigned int i = 0;
  if (msg_id == 0ULL) // if msg_id is zero, set fwd to "0"
  {
    fwd[0] = '0';
  }
  else
  {
    while (msg_id > 0)
    {
      rev[i++] = (msg_id % 10) + '0'; // divide by 10, convert the remainder to char
      msg_id /= 10; // divide by 10
    }
    unsigned int j = 0;
    while (i > 0)
    {
      fwd[j++] = rev[--i]; // reverse the order
    }
  }
  strcat(command, fwd);

  strcat(command, "*"); // Append the asterix

  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    swarm_m138_free_char(rev);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MT DELETED", "$MT ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_DELETE_TIMEOUT);

  swarm_m138_free_char(command);
  swarm_m138_free_char(fwd);
  swarm_m138_free_char(rev);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Delete all unsent messages
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::deleteAllTxMessages(void)
{
  char *command;
  char *response;
  char *scratchpad;
  Swarm_M138_Error_e err;
  uint16_t msgTotal = 0;

  err = getUnsentMessageCount(&msgTotal); // Get the message count so we know how many messages to expect
  if (err != SWARM_M138_ERROR_SUCCESS)
    return (err);

  if (_printDebug == true)
  {
    _debugPort->print(F("deleteAllTxMessages: msgTotal is "));
    _debugPort->println(msgTotal);
  }

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 9);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 9); // Clear it
  sprintf(command, "%s D=U*", SWARM_M138_COMMAND_MSG_TX_MGMT); // Copy the command, add the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  scratchpad = swarm_m138_alloc_char(16); // Create a scratchpad to hold the expectedResponse
  if (scratchpad == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(response);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(scratchpad, 0, 16); // Clear it

  sprintf(scratchpad, "$MT %d*", msgTotal); // Create the expected response

  err = sendCommandWithResponse(command, scratchpad, "$MT ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_DELETE_TIMEOUT);

  swarm_m138_free_char(command);
  swarm_m138_free_char(response);
  swarm_m138_free_char(scratchpad);
  return (err);
}

/**************************************************************************/
/*!
    @brief  List the unsent message with the specified ID
    @param  msg_id
            The ID of the message to be listed
    @param  asciiHex
            A pointer to a char array to hold the message
    @param  len
            The maximum message length which asciiHex can hold
    @param  epoch
            Optional: a pointer to a uint32_t to hold the epoch at which the modem received the message
    @param  appID
            Optional: a pointer to a uint16_t to hold the message appID if there is one
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::listTxMessage(uint64_t msg_id, char *asciiHex, size_t len, uint32_t *epoch, uint16_t *appID)
{
  char *command;
  char *response;
  char *fwd;
  char *rev;
  char *responseStart;
  char *responseEnd = NULL;
  Swarm_M138_Error_e err;

  memset(asciiHex, 0, len); // Clear the char array

  // Allocate memory for the command, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 3 + 20 + 5);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 3 + 20 + 5); // Clear it

  // Allocate memory for the scratchpads
  fwd = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (fwd == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(fwd, 0, 21); // Clear it
  rev = swarm_m138_alloc_char(21); // Up to 20 digits plus null
  if (rev == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(rev, 0, 21); // Clear it

  sprintf(command, "%s L=", SWARM_M138_COMMAND_MSG_TX_MGMT); // Copy the command

  // Add the 64-bit message ID
  // Based on printLLNumber by robtillaart
  // https://forum.arduino.cc/index.php?topic=143584.msg1519824#msg1519824
  unsigned int i = 0;
  if (msg_id == 0ULL) // if msg_id is zero, set fwd to "0"
  {
    fwd[0] = '0';
  }
  else
  {
    while (msg_id > 0)
    {
      rev[i++] = (msg_id % 10) + '0'; // divide by 10, convert the remainder to char
      msg_id /= 10; // divide by 10
    }
    unsigned int j = 0;
    while (i > 0)
    {
      fwd[j++] = rev[--i]; // reverse the order
    }
  }
  strcat(command, fwd);

  strcat(command, "*"); // Append the asterix

  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(fwd);
    swarm_m138_free_char(rev);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$MT ", "$MT ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_READ_TIMEOUT);

  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    responseStart = strstr(response, "$MT "); // Find the start of the response
    if (responseStart != NULL)
    {
      responseEnd = strchr(responseStart, '*'); // Find the asterix
      if (responseEnd != NULL)
      {
        responseStart = strstr(responseStart, " AI="); // Check if the message has AI= at the start
        if (responseStart != NULL)
        {
          // Extract the appID if required
          if (appID != NULL)
          {
            int appID_i = 0;
            int ret = sscanf(responseStart, " AI=%d,", &appID_i);
            if (ret == 1)
            {
              *appID = (uint16_t)appID_i;
            }
          }

          responseStart = strchr(responseStart, ','); // Find the first comma (we know it is there)
          responseStart++; // Point to the first digit of the ASCII Hex
        }
        else // Message does not have AI= (Should be impossible with firmware >= v2)
        {
          responseStart = strstr(response, "$MT "); // Find the $MT again (we know it is there)
          responseStart += 4; // Point to the first digit of the ASCII Hex
        }
        
        if (responseStart != NULL)
        {
          size_t charsRead = 0;
          char c = *responseStart;
          while ((c != ',') && (responseStart < responseEnd) && (charsRead < len)) // Stop at the next comma
          {
            asciiHex[charsRead] = c; // Copy the char into asciiHex
            responseStart++; // Increment the pointer
            charsRead++; // Increment the counter
            c = *responseStart; // Get the next char
          }

          if (epoch != NULL) // Check if epoch is NULL
          {
            responseStart++;
            responseStart = strchr(responseStart, ','); // Find the next comma
            if (responseStart != NULL)
            {
              responseStart++; // Point to the first digit of the epoch
              uint32_t theEpoch = 0;
              char c = *responseStart;
              while ((c != '*') && (responseStart < responseEnd)) // Stop at the asterix
              {
                theEpoch *= 10;
                theEpoch += (uint32_t)(c - '0');
                responseStart++;
                c = *responseStart;
              }

              *epoch = theEpoch;
            }
          }
        }
        else
          err = SWARM_M138_ERROR_INVALID_FORMAT;
      }
      else
        err = SWARM_M138_ERROR_ERROR;
    }
    else
      err = SWARM_M138_ERROR_ERROR;
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(fwd);
  swarm_m138_free_char(rev);
  swarm_m138_free_char(response);
  return (err);
}

// ** listTxMessagesIDs is not supported with modem firmware >= v2.0.0 **
// 
// /**************************************************************************/
// /*!
//     @brief  List the IDs of all the unsent messages
//     @param  ids
//             A pointer to an array of uint64_t to hold the message IDs.
//             Call getUnsentMessageCount first so you know how many IDs to expect and allocate storage for them.
//             Listing all the messages through the backlog could blow up the memory,
//             so this function reads the returned text one byte at a time and extracts the IDs from that.
//     @param  maxCount
//             Stop after listing the IDs for this many messages
//             Set this to the size of your uint64_t array
//     @return SWARM_M138_ERROR_SUCCESS if successful
//             SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
//             SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
//             SWARM_M138_ERROR_ERROR if unsuccessful
// */
// /**************************************************************************/
// Swarm_M138_Error_e SWARM_M138::listTxMessagesIDs(uint64_t *ids, uint16_t maxCount)
// {
//   char *command;
//   char *response;
//   char *rxBuff;
//   char *idStart;
//   char *idEnd;
//   Swarm_M138_Error_e err;
//   uint16_t msgTotal = 0, msgCount = 0;

//   if (maxCount == 0) // Quit if maxCount is zero. It must be at least one.
//     return (SWARM_M138_ERROR_ERROR);

//   // Calculate the "size of" each element in the array of uint64_t
//   // Note: sizeof(uint64_t) doesn't seem to give the correct result?
//   uint64_t test[2];
//   size_t sizeOfUint64Array = &test[1] - &test[0];

//   if (_printDebug == true)
//   {
//     _debugPort->print(F("listTxMessagesIDs: sizeOfUint64Array is "));
//     _debugPort->println(sizeOfUint64Array);
//   }

//   err = getUnsentMessageCount(&msgTotal); // Get the message count so we know how many messages to expect
//   if (err != SWARM_M138_ERROR_SUCCESS)
//     return (err);

//   if (_printDebug == true)
//   {
//     _debugPort->print(F("listTxMessagesIDs: msgTotal is "));
//     _debugPort->println(msgTotal);
//   }

//   if (msgTotal == 0) // Quit if there are no messages in the buffer
//     return (SWARM_M138_ERROR_ERROR);

//   // Allocate memory for the response
//   response = swarm_m138_alloc_char(_RxBuffSize);
//   if (response == NULL)
//   {
//     return(SWARM_M138_ERROR_MEM_ALLOC);
//   }
//   memset(response, 0, _RxBuffSize); // Clear it

//   // Allocate memory for the command, asterix, checksum bytes, \n and \0
//   command = swarm_m138_alloc_char(strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 9);
//   if (command == NULL)
//   {
//     swarm_m138_free_char(response);
//     return (SWARM_M138_ERROR_MEM_ALLOC);
//   }
//   memset(command, 0, strlen(SWARM_M138_COMMAND_MSG_TX_MGMT) + 9); // Clear it
//   sprintf(command, "%s L=U*", SWARM_M138_COMMAND_MSG_TX_MGMT); // Copy the command, add the asterix
//   addChecksumLF(command); // Add the checksum bytes and line feed

//   // if (_printDebug == true)
//   // {
//   //   _debugPort->println(F("listTxMessagesIDs: ====>"));
//   //   _debugPort->flush();
//   // }

//   sendCommand(command); // Send the command

//   swarm_m138_free_char(command); // Free command now - we are done with it

//   unsigned long startTime = millis();
//   bool keepGoing = true;

//   // Keep checking incoming chars for up to SWARM_M138_MESSAGE_ID_TIMEOUT secs total
//   while ((millis() < (startTime + SWARM_M138_MESSAGE_ID_TIMEOUT)) && keepGoing)
//   {
//     int hwAvail = hwAvailable();

//     if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
//     {
//       rxBuff = swarm_m138_alloc_char((size_t)(hwAvail + 1)); // Allocate memory to hold the data
//       if (rxBuff != NULL)
//       {
//         memset(rxBuff, 0, (size_t)(hwAvail + 1)); // Clear it
//         int charsRead = hwReadChars(rxBuff, hwAvail); // Read the characters

//         // if (_printDebug == true)
//         //   _debugPort->print(rxBuff);

//         for (int i = 0; i < charsRead; i++) // Go through a character at a time
//         {
//           char cc[2];
//           cc[0] = rxBuff[i];
//           cc[1] = 0;
//           if (strlen(response) < _RxBuffSize) // Check if response is full
//           {
//             strcat(response, cc); // Copy a single character into response
//             if (cc[0] == '\n') // Check if this is a newline
//             {
//               idStart = strstr(response, "$MT "); // Check for $MT
//               if (idStart != NULL)
//               {
//                 idStart = strstr(idStart, " AI="); // Check if the message has AI= at the start
//                 if (idStart != NULL)
//                 {
//                   idStart = strchr(idStart, ','); // Find the first comma
//                   if (idStart != NULL)
//                   {
//                     idStart++;
//                     idStart = strchr(idStart, ','); // Find the second comma
//                   }
//                 }
//                 else // Message does not have AI=
//                 {
//                   idStart = strstr(response, "$MT "); // Find the $MT again
//                   if (idStart != NULL)
//                   {
//                     idStart = strchr(idStart, ','); // Find the first comma
//                   }
//                 }
                
//                 if (idStart != NULL)
//                 {
//                   idStart++; // Point to the first digit of the message ID
//                   idEnd = strchr(idStart, ','); // Find the second/third comma
//                   if (idEnd != NULL)
//                   {
//                     // We have (hopefully) a valid ID to extract
//                     uint64_t theID = 0;
//                     char c = *idStart;
//                     while ((c != ',') && (idStart < idEnd))
//                     {
//                       // if (_printDebug == true)
//                       //   _debugPort->write(c);
//                       theID *= 10;
//                       theID += (uint64_t)(c - '0');
//                       idStart++;
//                       c = *idStart;
//                     }
//                     *(ids + (sizeOfUint64Array * (size_t)msgCount)) = theID; // Store the extracted ID
//                     msgCount++; // Increment the count
//                     if (_printDebug == true)
//                     {
//                       _debugPort->println();
//                       _debugPort->print(F("listTxMessagesIDs: msgCount is "));
//                       _debugPort->println(msgCount);
//                     }
//                     if ((msgCount == msgTotal) || (msgCount == maxCount))
//                       keepGoing = false; // Stop if we have reached msgTotal or maxCount
//                   }
//                 }
//               }
//               else
//               {
//                 // We got a \n but failed to find $MT.
//                 // So, let's be nice and copy whatever this message is into the backlog if possible
//                 size_t backlogLength = strlen((const char *)_swarmBacklog);
//                 size_t msgLen = strlen((const char *)response);
//                 if ((backlogLength + msgLen) < _RxBuffSize)
//                 {
//                   memcpy(&_swarmBacklog[backlogLength], response, msgLen);
//                 }
//               }
//               memset(response, 0, _RxBuffSize); // Clear the response
//             }
//           }
//           else // Response is full... We are stuck... And we are still in the for loop...
//           {
//             swarm_m138_free_char(rxBuff);
//             swarm_m138_free_char(response);
//             return (SWARM_M138_ERROR_ERROR);
//           }
//         }
//         swarm_m138_free_char(rxBuff); // Free the buffer
//       }
//       else // The memory allocation failed
//       {
//         err = SWARM_M138_ERROR_MEM_ALLOC;
//         keepGoing = false;
//       }
//     }
//     else
//       delay(1);
//   }

//   swarm_m138_free_char(response);

//   // if (_printDebug == true)
//   // {
//   //   _debugPort->println(F("listTxMessagesIDs: <===="));
//   //   _debugPort->flush();
//   // }

//   return (err);
// }

/**************************************************************************/
/*!
    @brief  Queue a printable text message for transmission
    @param  data
            The message as printable ASCII characters (0x20 to 0x7E, space to ~)
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitText(const char *data, uint64_t *msg_id)
{
  return (transmitText(data, msg_id, false, 0, false, 0, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a printable text message for transmission with an appID
    @param  data
            The message as printable ASCII characters (0x20 to 0x7E, space to ~)
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  appID
            The application ID: 0 to 64999
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitText(const char *data, uint64_t *msg_id, uint16_t appID)
{
  return (transmitText(data, msg_id, true, appID, false, 0, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a printable text message for transmission with a hold duration
    @param  data
            The message as printable ASCII characters (0x20 to 0x7E, space to ~)
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  hold
            The hold duration in seconds: 60 to 34819200 (13 months)
            The message expires if it has not been transmitted within the hold duration
            The default hold duration is 172800 seconds (48 hours)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitTextHold(const char *data, uint64_t *msg_id, uint32_t hold)
{
  return (transmitText(data, msg_id, false, 0, true, hold, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a printable text message for transmission with a hold duration and an appID
    @param  data
            The message as printable ASCII characters (0x20 to 0x7E, space to ~)
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  hold
            The hold duration in seconds: 60 to 34819200 (13 months)
            The message expires if it has not been transmitted within the hold duration
            The default hold duration is 172800 seconds (48 hours)
    @param  appID
            The application ID: 0 to 64999
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitTextHold(const char *data, uint64_t *msg_id, uint32_t hold, uint16_t appID)
{
  return (transmitText(data, msg_id, true, appID, true, hold, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a printable text message for transmission with an expiry time (epoch)
    @param  data
            The message as printable ASCII characters (0x20 to 0x7E, space to ~)
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  epoch
            The second date after which the message will be expired if it has not been sent.
            1577836800 to 2147483647
            (2020-01-01 00:00:00 to 2038-01-19 03:14:07)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitTextExpire(const char *data, uint64_t *msg_id, uint32_t epoch)
{
  return (transmitText(data, msg_id, false, 0, false, 0, true, epoch));
}

/**************************************************************************/
/*!
    @brief  Queue a printable text message for transmission with an expiry time (epoch) and an appID
    @param  data
            The message as printable ASCII characters (0x20 to 0x7E, space to ~)
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  epoch
            The second date after which the message will be expired if it has not been sent.
            1577836800 to 2147483647
            (2020-01-01 00:00:00 to 2038-01-19 03:14:07)
    @param  appID
            The application ID: 0 to 64999
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitTextExpire(const char *data, uint64_t *msg_id, uint32_t epoch, uint16_t appID)
{
  return (transmitText(data, msg_id, true, appID, false, 0, true, epoch));
}

// Queue a text message for transmission
// Return the allocated message ID in msg_id
Swarm_M138_Error_e SWARM_M138::transmitText(const char *data, uint64_t *msg_id, bool useAppID, uint16_t appID,
                                            bool useHold, uint32_t hold, bool useEpoch, uint32_t epoch)
{
  char *command;
  char *response;
  char *scratchpad;
  Swarm_M138_Error_e err;

  // Calculate the possible message length
  size_t msgLen = strlen(SWARM_M138_COMMAND_TX_DATA); // $TD
  msgLen += 1; // Space
  if (useAppID) msgLen += 3 + 5 + 1; // AI=65535,
  if (useHold) msgLen += 3 + 8 + 1; // HD=34819200,
  if (useEpoch) msgLen += 3 + 10 + 1; // ET=2147483647,
  msgLen += 2 + strlen(data); // Quotes plus the message itself
  msgLen += 5; // asterix, checksum chars, line feed, null

  // Allocate memory for the command, message, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(msgLen);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, msgLen); // Clear it

  // Allocate memory for the scratchpad
  scratchpad = swarm_m138_alloc_char(16);
  if (scratchpad == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(scratchpad, 0, 16); // Clear it

  sprintf(command, "%s ", SWARM_M138_COMMAND_TX_DATA); // Copy the command. Append the space
  if (useAppID)
  {
    strcat(command, "AI=");
    sprintf(scratchpad, "%d", appID);
    strcat(command, scratchpad);
    strcat(command, ",");
  }
  if (useHold)
  {
    strcat(command, "HD=");
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    sprintf(scratchpad, "%d", hold);
#else
    sprintf(scratchpad, "%ld", hold);
#endif
    strcat(command, scratchpad);
    strcat(command, ",");
  }
  if (useEpoch)
  {
    strcat(command, "ET=");
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    sprintf(scratchpad, "%d", epoch);
#else
    sprintf(scratchpad, "%ld", epoch);
#endif
    strcat(command, scratchpad);
    strcat(command, ",");
  }
  strcat(command, "\""); // Append the quote  
  strcat(command, data); // Append the message
  strcat(command, "\"*"); // Append the quote and asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(scratchpad);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$TD OK,", "$TD ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_TRANSMIT_TIMEOUT);

  if (err == SWARM_M138_ERROR_SUCCESS) // Check if we got $TD OK
  {
    char *idStart = strstr(response, "$TD OK,");
    if (idStart != NULL)
    {
      char *idEnd = strchr(idStart, '*'); // Look for the asterix
      if (idEnd != NULL)
      {
        uint64_t theID = 0;

        idStart += 7; // Point at the first digit of the ID

        while (idStart < idEnd)
        {
          theID *= 10;
          theID += (uint64_t)((*idStart) - '0'); // Add each digit to theID
          idStart++;
        }

        *msg_id = theID;
      }
    }
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(scratchpad);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Queue a binary message for transmission
    @param  data
            The binary message
    @param  len
            The length of the binary message in bytes
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitBinary(const uint8_t *data, size_t len, uint64_t *msg_id)
{
  return (transmitBinary(data, len, msg_id, false, 0, false, 0, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a binary message for transmission with an appID
    @param  data
            The binary message
    @param  len
            The length of the binary message in bytes
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  appID
            The application ID: 0 to 64999
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitBinary(const uint8_t *data, size_t len, uint64_t *msg_id, uint16_t appID)
{
  return (transmitBinary(data, len, msg_id, true, appID, false, 0, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a binary message for transmission with a hold duration
    @param  data
            The binary message
    @param  len
            The length of the binary message in bytes
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  hold
            The hold duration in seconds: 60 to 34819200 (13 months)
            The message expires if it has not been transmitted within the hold duration
            The default hold duration is 172800 seconds (48 hours)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitBinaryHold(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t hold)
{
  return (transmitBinary(data, len, msg_id, false, 0, true, hold, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a binary message for transmission with a hold duration and an appID
    @param  data
            The binary message
    @param  len
            The length of the binary message in bytes
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  hold
            The hold duration in seconds: 60 to 34819200 (13 months)
            The message expires if it has not been transmitted within the hold duration
            The default hold duration is 172800 seconds (48 hours)
    @param  appID
            The application ID: 0 to 64999
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitBinaryHold(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t hold, uint16_t appID)
{
  return (transmitBinary(data, len, msg_id, true, appID, true, hold, false, 0));
}

/**************************************************************************/
/*!
    @brief  Queue a binary message for transmission with an expiry time (epoch)
    @param  data
            The binary message
    @param  len
            The length of the binary message in bytes
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  epoch
            The second date after which the message will be expired if it has not been sent.
            1577836800 to 2147483647
            (2020-01-01 00:00:00 to 2038-01-19 03:14:07)
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitBinaryExpire(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t epoch)
{
  return (transmitBinary(data, len, msg_id, false, 0, false, 0, true, epoch));
}

/**************************************************************************/
/*!
    @brief  Queue a binary message for transmission with an expiry time (epoch) and an appID
    @param  data
            The binary message
    @param  len
            The length of the binary message in bytes
    @param  msg_id
            A pointer to a uint64_t which will hold the assigned message ID
    @param  epoch
            The second date after which the message will be expired if it has not been sent.
            1577836800 to 2147483647
            (2020-01-01 00:00:00 to 2038-01-19 03:14:07)
    @param  appID
            The application ID: 0 to 64999
    @return SWARM_M138_ERROR_SUCCESS if successful
            SWARM_M138_ERROR_MEM_ALLOC if the memory allocation fails
            SWARM_M138_ERROR_ERR if a command ERR is received - error is returned in commandError
            SWARM_M138_ERROR_ERROR if unsuccessful
*/
/**************************************************************************/
Swarm_M138_Error_e SWARM_M138::transmitBinaryExpire(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t epoch, uint16_t appID)
{
  return (transmitBinary(data, len, msg_id, true, appID, false, 0, true, epoch));
}

// Queue a binary message for transmission
// Return the allocated message ID in msg_id
Swarm_M138_Error_e SWARM_M138::transmitBinary(const uint8_t *data, size_t len, uint64_t *msg_id, bool useAppID, uint16_t appID,
                                            bool useHold, uint32_t hold, bool useEpoch, uint32_t epoch)
{
  char *command;
  char *response;
  char *scratchpad;
  Swarm_M138_Error_e err;

  // Calculate the possible message length
  size_t msgLen = strlen(SWARM_M138_COMMAND_TX_DATA); // $TD
  msgLen += 1; // Space
  if (useAppID) msgLen += 3 + 5 + 1; // AI=65535,
  if (useHold) msgLen += 3 + 8 + 1; // HD=34819200,
  if (useEpoch) msgLen += 3 + 10 + 1; // ET=2147483647,
  msgLen += 2 * len; // The message length in ASCII Hex
  msgLen += 5; // asterix, checksum chars, line feed, null

  // Allocate memory for the command, message, asterix, checksum bytes, \n and \0
  command = swarm_m138_alloc_char(msgLen);
  if (command == NULL)
    return (SWARM_M138_ERROR_MEM_ALLOC);
  memset(command, 0, msgLen); // Clear it

  // Allocate memory for the scratchpad
  scratchpad = swarm_m138_alloc_char(16);
  if (scratchpad == NULL)
  {
    swarm_m138_free_char(command);
    return (SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(scratchpad, 0, 16); // Clear it

  sprintf(command, "%s ", SWARM_M138_COMMAND_TX_DATA); // Copy the command. Append the space
  if (useAppID)
  {
    strcat(command, "AI=");
    sprintf(scratchpad, "%d", appID);
    strcat(command, scratchpad);
    strcat(command, ",");
  }
  if (useHold)
  {
    strcat(command, "HD=");
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    sprintf(scratchpad, "%d", hold);
#else
    sprintf(scratchpad, "%ld", hold);
#endif
    strcat(command, scratchpad);
    strcat(command, ",");
  }
  if (useEpoch)
  {
    strcat(command, "ET=");
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    sprintf(scratchpad, "%d", epoch);
#else
    sprintf(scratchpad, "%ld", epoch);
#endif
    strcat(command, scratchpad);
    strcat(command, ",");
  }
  for (size_t i = 0; i < len; i++)
  {
    char c1 = (data[i] >> 4) + '0'; // Convert the MS nibble to ASCII
    if (c1 >= ':') c1 = c1 + 'A' - ':';
    char c2 = (data[i] & 0x0F) + '0'; // Convert the LS nibble to ASCII
    if (c2 >= ':') c2 = c2 + 'A' - ':';
    sprintf(scratchpad, "%c%c", c1, c2);
    strcat(command, scratchpad); // Append each data byte as an ASCII Hex char pair
  }
  strcat(command, "*"); // Append the asterix
  addChecksumLF(command); // Add the checksum bytes and line feed

  response = swarm_m138_alloc_char(_RxBuffSize); // Allocate memory for the response
  if (response == NULL)
  {
    swarm_m138_free_char(command);
    swarm_m138_free_char(scratchpad);
    return(SWARM_M138_ERROR_MEM_ALLOC);
  }
  memset(response, 0, _RxBuffSize); // Clear it

  err = sendCommandWithResponse(command, "$TD OK,", "$TD ERR", response, _RxBuffSize, SWARM_M138_MESSAGE_TRANSMIT_TIMEOUT);

  if (err == SWARM_M138_ERROR_SUCCESS) // Check if we got $TD OK
  {
    char *idStart = strstr(response, "$TD OK,");
    if (idStart != NULL)
    {
      char *idEnd = strchr(idStart, '*'); // Look for the asterix
      if (idEnd != NULL)
      {
        uint64_t theID = 0;

        idStart += 7; // Point at the first digit of the ID

        while (idStart < idEnd)
        {
          theID *= 10;
          theID += (uint64_t)((*idStart) - '0'); // Add each digit to theID
          idStart++;
        }

        *msg_id = theID;
      }
    }
  }

  swarm_m138_free_char(command);
  swarm_m138_free_char(scratchpad);
  swarm_m138_free_char(response);
  return (err);
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $DT Date Time message
    @param  swarmDateTimeCallback
            The address of the function to be called when an unsolicited $DT message arrives
*/
/**************************************************************************/
void SWARM_M138::setDateTimeCallback(void (*swarmDateTimeCallback)(const Swarm_M138_DateTimeData_t *dateTime))
{
  _swarmDateTimeCallback = swarmDateTimeCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $GJ jamming indication message
    @param  swarmGpsJammingCallback
            The address of the function to be called when an unsolicited $GJ message arrives
*/
/**************************************************************************/
void SWARM_M138::setGpsJammingCallback(void (*swarmGpsJammingCallback)(const Swarm_M138_GPS_Jamming_Indication_t *jamming))
{
  _swarmGpsJammingCallback = swarmGpsJammingCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $GN geospatial information message
    @param  swarmGeospatialCallback
            The address of the function to be called when an unsolicited $GN message arrives
*/
/**************************************************************************/
void SWARM_M138::setGeospatialInfoCallback(void (*swarmGeospatialCallback)(const Swarm_M138_GeospatialData_t *info))
{
  _swarmGeospatialCallback = swarmGeospatialCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $GS GPS fix quality message
    @param  swarmGpsFixQualityCallback
            The address of the function to be called when an unsolicited $GS message arrives
*/
/**************************************************************************/
void SWARM_M138::setGpsFixQualityCallback(void (*swarmGpsFixQualityCallback)(const Swarm_M138_GPS_Fix_Quality_t *fixQuality))
{
  _swarmGpsFixQualityCallback = swarmGpsFixQualityCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $PW power status message
    @param  swarmPowerStatusCallback
            The address of the function to be called when an unsolicited $PW message arrives
*/
/**************************************************************************/
void SWARM_M138::setPowerStatusCallback(void (*swarmPowerStatusCallback)(const Swarm_M138_Power_Status_t *power))
{
  _swarmPowerStatusCallback = swarmPowerStatusCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $RT receive test message
    @param  swarmReceiveTestCallback
            The address of the function to be called when an unsolicited $RT message arrives
*/
/**************************************************************************/
void SWARM_M138::setReceiveTestCallback(void (*swarmReceiveTestCallback)(const Swarm_M138_Receive_Test_t *rxTest))
{
  _swarmReceiveTestCallback = swarmReceiveTestCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $M138 modem status messages
    @param  swarmModemStatusCallback
            The address of the function to be called when an unsolicited $M138 message arrives
*/
/**************************************************************************/
void SWARM_M138::setModemStatusCallback(void (*swarmModemStatusCallback)(Swarm_M138_Modem_Status_e status, const char *debugOrError))
{
  _swarmModemStatusCallback = swarmModemStatusCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $SL WAKE sleep mode messages
    @param  swarmSleepWakeCallback
            The address of the function to be called when an unsolicited $SL WAKE message arrives
*/
/**************************************************************************/
void SWARM_M138::setSleepWakeCallback(void (*swarmSleepWakeCallback)(Swarm_M138_Wake_Cause_e cause))
{
  _swarmSleepWakeCallback = swarmSleepWakeCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for the $RD receive data message
    @param  swarmReceiveMessageCallback
            The address of the function to be called when an unsolicited $RD message arrives
*/
/**************************************************************************/
void SWARM_M138::setReceiveMessageCallback(void (*swarmReceiveMessageCallback)(const uint16_t *appID, const int16_t *rssi,
                                           const int16_t *snr, const int16_t *fdev, const char *asciiHex))
{
  _swarmReceiveMessageCallback = swarmReceiveMessageCallback;
}

/**************************************************************************/
/*!
    @brief  Set up the callback for $TD SENT messages
    @param  swarmTransmitDataCallback
            The address of the function to be called when an unsolicited $TD SENT message arrives
*/
/**************************************************************************/
void SWARM_M138::setTransmitDataCallback(void (*swarmTransmitDataCallback)(const int16_t *rssi_sat, const int16_t *snr,
                                         const int16_t *fdev, const uint64_t *id))
{
  _swarmTransmitDataCallback = swarmTransmitDataCallback;
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
    case SWARM_M138_MODEM_STATUS_BOOT_DEVICEID:
      return "BOOT DEVICEID (Device ID of the modem)";
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
    case SWARM_M138_MODEM_STATUS_BOOT_RESTART:
      return "BOOT RESTART (Modem is restarting)";
      break;
    case SWARM_M138_MODEM_STATUS_BOOT_SHUTDOWN:
      return "BOOT SHUTDOWN (Modem has shutdown. Disconnect power to restart)";
      break;
    case SWARM_M138_MODEM_STATUS_DATETIME:
      return "DATETIME (GPS has acquired a valid date/time reference)";
      break;
    case SWARM_M138_MODEM_STATUS_POSITION:
      return "POSITION (GPS has acquired a valid position 3D fix)";
      break;
    case SWARM_M138_MODEM_STATUS_DEBUG:
      return "DEBUG (Debug message)";
      break;
    case SWARM_M138_MODEM_STATUS_ERROR:
      return "ERROR (Error message)";
      break;
    case SWARM_M138_MODEM_STATUS_UNKNOWN:
      return "UNKNOWN (undocumented)";
      break;
    default:
      return "INVALID";
      break;
  }
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
    case SWARM_M138_ERROR_INVALID_FORMAT:
      return "Indicates the command response format was invalid: missing $ or *; non-ASCII checksum";
      break;
    case SWARM_M138_ERROR_INVALID_CHECKSUM:
      return "Indicates the command response checksum was invalid";
      break;
    case SWARM_M138_ERROR_INVALID_RATE:
      return "The requested message rate was invalid";
      break;
    case SWARM_M138_ERROR_INVALID_MODE:
      return "The requested GPIO1 pin mode was invalid";
      break;
    case SWARM_M138_ERROR_ERR:
      return "Command input error (ERR)";
      break;
  }

  return "UNKNOWN";
}

/**************************************************************************/
/*!
    @brief  Convert command error into a printable description
    @param  ERR
            The command error as const char *
    @return A pointer to the command error description in string (const char) format
*/
/**************************************************************************/
const char *SWARM_M138::commandErrorString(const char *ERR)
{
  if (strstr(ERR, "BADPARAMLENGTH") != NULL)
    return "A parameter has an incorrect length";
  if (strstr(ERR, "BADPARAMVALUE") != NULL)
    return "A parameter has a value that is out of range";
  if (strstr(ERR, "BADPARAM") != NULL)
    return "Unrecognizable parameter after command";
  if (strstr(ERR, "INVALIDCHAR") != NULL)
    return "A parameter has an invalid character";
  if (strstr(ERR, "NOTIMPLEMENTED") != NULL)
    return "The command is not recognized as valid";
  if (strstr(ERR, "PARAMMISSING") != NULL)
    return "A required parameter is missing";
  if (strstr(ERR, "PARAMDUPLICATE") != NULL)
    return "A parameter has been duplicated";
  if (strstr(ERR, "DBX_INVMSGID") != NULL)
    return "Messages Management : invalid message ID";
  if (strstr(ERR, "DBX_NOMORE") != NULL)
    return "Messages Management : no messages found";
  if (strstr(ERR, "TIMENOTSET") != NULL)
    return "Time not yet set from GPS";
  if (strstr(ERR, "NOCOMMAND") != NULL)
    return "Sleep Mode : No S or U partameter";
  if (strstr(ERR, "NOTIME") != NULL)
    return "Sleep Mode : attempt to sleep before time is set";
  if (strstr(ERR, "BADAPPID") != NULL)
    return "Transmit Data : invalid application ID";
  if (strstr(ERR, "BADDATA") != NULL)
    return "Transmit Data : Message has odd number or non-hex characters when sending data as hexadecimal";
  if (strstr(ERR, "BADEXPIRETIME") != NULL)
    return "Transmit Data : Invalid hold time";
  if (strstr(ERR, "HOLDTIMEEXPIRED") != NULL)
    return "Transmit Data : Unable to send within requested hold time";
  if (strstr(ERR, "NODEVICEID") != NULL)
    return "Transmit Data : The Swarm device ID has not yet been set - contact Swarm Support";
  if (strstr(ERR, "NOSPACE") != NULL)
    return "Transmit Data : No space for message";
  if (strstr(ERR, "DBXTOHIVEFULL") != NULL)
    return "Transmit Data : Queue for queued messages is full. Maximum of 2048 messages may be held in the queue";
  if (strstr(ERR, "TOOLONG") != NULL)
    return "Transmit Data : Message is too large to send";

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

  char *asterix = strchr(dollar, '*'); // Find the *

  if (asterix == NULL) // Return now if the * was not found
    return;

  // Check for a second asterix ($MM C=**)
  if (*(asterix + 1) == '*')
    asterix++;

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

// Check if the response / message checksum is valid
Swarm_M138_Error_e SWARM_M138::checkChecksum(char *startPosition)
{
  char *dollar = strchr(startPosition, '$'); // Find the $

  if (dollar == NULL) // Return now if the $ was not found
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("checkChecksum: $ not found!"));
    }
    return (SWARM_M138_ERROR_INVALID_FORMAT);
  }

  char *asterix = strchr(dollar, '*'); // Find the *

  if (asterix == NULL) // Return now if the * was not found
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("checkChecksum: * not found!"));
    }
    return (SWARM_M138_ERROR_INVALID_FORMAT);
  }

  char checksum = 0;

  dollar++; // Point to the char after the $

  while (dollar < asterix) // Calculate the checksum
  {
    checksum ^= *dollar;
    dollar++;
  }

  char expectedChecksum;

  char checksumChar = *(asterix + 1); // Get the first checksum character

  if ((checksumChar >= '0') && (checksumChar <= '9')) // Convert to binary
    expectedChecksum = (checksumChar - '0') << 4;
  else if ((checksumChar >= 'a') && (checksumChar <= 'f'))
    expectedChecksum = (checksumChar + 10 - 'a') << 4;
  else if ((checksumChar >= 'A') && (checksumChar <= 'F'))
    expectedChecksum = (checksumChar + 10 - 'A') << 4;
  else
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("checkChecksum: invalid checksum char 1"));
    }
    return (SWARM_M138_ERROR_INVALID_FORMAT);
  }

  checksumChar = *(asterix + 2); // Get the second checksum character

  if ((checksumChar >= '0') && (checksumChar <= '9')) // Convert to binary
    expectedChecksum |= (checksumChar - '0');
  else if ((checksumChar >= 'a') && (checksumChar <= 'f'))
    expectedChecksum |= (checksumChar + 10 - 'a');
  else if ((checksumChar >= 'A') && (checksumChar <= 'F'))
    expectedChecksum |= (checksumChar + 10 - 'A');
  else
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("checkChecksum: invalid checksum char 2"));
    }
    return (SWARM_M138_ERROR_INVALID_FORMAT);
  }

  if (checksum != expectedChecksum)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("checkChecksum: invalid checksum"));
    }
    return (SWARM_M138_ERROR_INVALID_CHECKSUM);
  }

  return (SWARM_M138_ERROR_SUCCESS);
}

// Extract the command error
Swarm_M138_Error_e SWARM_M138::extractCommandError(char *startPosition)
{
    memset(commandError, 0, SWARM_M138_MAX_CMD_ERROR_LEN); // Clear any existing error

    char *errorAt = strstr(startPosition, "ERR,"); // Find the ERR,

    if (errorAt == NULL)
      return (SWARM_M138_ERROR_ERROR);

    errorAt += 4; // Point to the start of the actual error message

    char *asterix = strchr(errorAt, '*'); // Find the *

    if (asterix == NULL)
      return (SWARM_M138_ERROR_ERROR);

    int errorLen = 0;
    while ((errorAt < asterix) && (errorLen < (SWARM_M138_MAX_CMD_ERROR_LEN - 1))) // Leave a NULL on the end
    {
      commandError[errorLen] = *errorAt;
      errorAt++;
      errorLen++;
    }

    return (SWARM_M138_ERROR_SUCCESS);
}

// Send a command. Check for a response.
// Return true if expectedResponseStart is seen in the data followed by a \n
Swarm_M138_Error_e SWARM_M138::sendCommandWithResponse(
    const char *command, const char *expectedResponseStart, const char *expectedErrorStart,
    char *responseDest, size_t destSize, unsigned long commandTimeout)
{
  if (_printDebug == true)
    _debugPort->println(F("sendCommandWithResponse: ====>"));

  sendCommand(command); //Sending command needs to dump data to backlog buffer as well.

  Swarm_M138_Error_e err = waitForResponse(expectedResponseStart, expectedErrorStart, responseDest, destSize, commandTimeout);

  if (_printDebug == true)
    _debugPort->println(F("sendCommandWithResponse: <===="));

  return (err);
}

void SWARM_M138::sendCommand(const char *command)
{
  //Spend up to _rxWindowMillis milliseconds copying any incoming serial data into the backlog
  unsigned long timeIn = millis();
  int hwAvail = hwAvailable();
  if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
  {
    size_t backlogLength = strlen((const char *)_swarmBacklog);
    while (((millis() - timeIn) < _rxWindowMillis) && ((backlogLength + hwAvail) < _RxBuffSize)) //May need to escape on newline?
    {
      if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        backlogLength += hwReadChars((char *)&_swarmBacklog[backlogLength], hwAvail);
        timeIn = millis();
      }
      else
        delay(1);
      hwAvail = hwAvailable();
    }
  }

  if (_printDebug == true)
  {
    _debugPort->print(F("sendCommand: Command: "));
    _debugPort->println(command);
  }

  //Now send the command
  hwPrint(command);
}

Swarm_M138_Error_e SWARM_M138::waitForResponse(const char *expectedResponseStart, const char *expectedErrorStart,
                                               char *responseDest, size_t destSize, unsigned long timeout)
{
  unsigned long timeIn;
  bool found = false;
  size_t destIndex = 0;
  bool responseStartSeen = false, errorStartSeen = false;
  int responseIndex = 0, errorIndex = 0;
  size_t responseStartedAt = 0, errorStartedAt = 0;
  Swarm_M138_Error_e err = SWARM_M138_ERROR_ERROR;

  bool printedSomething = false;

  timeIn = millis();

  while ((!found) && ((timeIn + timeout) > millis()))
  {
    size_t hwAvail = hwAvailable();
    if (hwAvail > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      if ((destIndex + hwAvail) < destSize) // Check there is room to store the response (with a null on the end!)
      {
        size_t bytesRead = hwReadChars((char *)&responseDest[destIndex], hwAvail);

        if (_printDebug == true)
        {
          if (printedSomething == false)
          {
            _debugPort->print(F("waitForResponse: "));
            printedSomething = true;
          }
          _debugPort->print((const char *)&responseDest[destIndex]);
        }

        // Check each character to see if it is the expected resonse or error
        for (size_t chrPtr = destIndex; chrPtr < (destIndex + bytesRead); chrPtr++)
        {
          char c = responseDest[chrPtr]; // Check each character

          if (c == expectedResponseStart[responseIndex])
          {
            if ((responseIndex == 0) && (responseStartSeen == false))
              responseStartedAt = chrPtr; // Record where the (possible) response started
            if (++responseIndex == (int)strlen(expectedResponseStart))
            {
              responseStartSeen = true;
            }
          }
          else
          {
            responseIndex = 0;
          }

          if (expectedErrorStart != NULL)
          {
            if (c == expectedErrorStart[errorIndex])
            {
              if ((errorIndex == 0) && (errorStartSeen == false))
                errorStartedAt = chrPtr; // Record where the (possible) error started
              if (++errorIndex == (int)strlen(expectedErrorStart))
              {
                errorStartSeen = true;
              }
            }
            else
            {
              errorIndex = 0;
            }
          }

          // Set found to true if we have seen expectedResponseStart or expectedErrorStart _and_ have received a \n
          if ((responseStartSeen || errorStartSeen) && (c == '\n'))
            found = true;
        }

        // Now also copy the response into the backlog, if there is room
        // _swarmBacklog is a global array that holds the backlog of any events
        // that came in while waiting for response. To be processed later within checkUnsolicitedMsg().
        // Note: the expectedResponse or expectedError will also be added to the backlog.
        // Everything in the backlog is 'printable'. So it is OK to use strlen - so long as there is a
        // \0 at the end of the buffer!
        size_t backlogLength = strlen((const char *)_swarmBacklog);

        if ((backlogLength + bytesRead) < _RxBuffSize) // Is there room to store the new data?
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
            _debugPort->println(F("waitForResponse: Panic! _swarmBacklog is full!"));
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
          _debugPort->println(F("waitForResponse: Panic! responseDest is full!"));
        }
      }
    }
    else
      delay(1);
  }

  if (_printDebug == true)
    if (printedSomething)
      _debugPort->println();

  if (found == true)
  {
    if (errorStartSeen) // Error needs priority over response as response is often the beginning of error!
    {
      // if (_printDebug == true)
      // {
      //   _debugPort->print(F("waitForResponse: errorStart: "));
      //   _debugPort->println((char *)&_swarmBacklog[errorStartedAt]);
      // }
      err = checkChecksum((char *)&_swarmBacklog[errorStartedAt]);
      if (err == SWARM_M138_ERROR_SUCCESS)
      {
        extractCommandError((char *)&_swarmBacklog[errorStartedAt]);
        err = SWARM_M138_ERROR_ERR;
      }
    }
    else if (responseStartSeen)
    {
      // if (_printDebug == true)
      // {
      //   _debugPort->print(F("waitForResponse: responseStart: "));
      //   _debugPort->println((char *)&_swarmBacklog[responseStartedAt]);
      // }
      err = checkChecksum((char *)&_swarmBacklog[responseStartedAt]);
    }
  }
  else
    err = SWARM_M138_ERROR_TIMEOUT;

  pruneBacklog(); // Prune any incoming non-actionable URC's and responses/errors from the backlog

  return (err);
}

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
char *SWARM_M138::swarm_m138_alloc_char(size_t num)
{
  return ((char *)new char[num]);
}
void SWARM_M138::swarm_m138_free_char(char *freeMe)
{
  delete[] freeMe;
}

//This prunes the backlog of non-actionable events. If new actionable events are added, you must modify the if statement.
void SWARM_M138::pruneBacklog()
{
  char *event;

  char *_pruneBuffer = swarm_m138_alloc_char(_RxBuffSize);
  if (_pruneBuffer == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("pruneBacklog: not enough memory for _pruneBuffer! Clearing the backlog. Sorry!"));
    memset(_swarmBacklog, 0, _RxBuffSize); //Clear out backlog buffer.
    return;
  }
  memset(_pruneBuffer, 0, _RxBuffSize); // Clear the _pruneBuffer

  char *preservedEvent;
  event = strtok_r(_swarmBacklog, "\n", &preservedEvent); // Look for an 'event' - something ending in \n

  while (event != NULL) //If event is actionable, add it to pruneBuffer.
  {
    // These are the events we want to keep so they can be processed by poll / checkUnsolicitedMsg
    if ((strstr(event, "$DT ") != NULL)
        || (strstr(event, "$GJ ") != NULL)
        || (strstr(event, "$GN ") != NULL)
        || (strstr(event, "$GS ") != NULL)
        || (strstr(event, "$PW ") != NULL)
        || (strstr(event, "$RD ") != NULL)
        || (strstr(event, "$RT ") != NULL)
        || (strstr(event, "$SL ") != NULL)
        || (strstr(event, "$M138 ") != NULL)
        || (strstr(event, "$TD ") != NULL))
    {
      strcat(_pruneBuffer, event); // The URCs are all readable text so using strcat is OK
      strcat(_pruneBuffer, "\n"); // strtok blows away delimiter, but we want that for later.
    }

    event = strtok_r(NULL, "\n", &preservedEvent); // Walk though any remaining events
  }

  memset(_swarmBacklog, 0, _RxBuffSize); //Clear out backlog buffer.
  memcpy(_swarmBacklog, _pruneBuffer, strlen(_pruneBuffer)); //Copy the pruned buffer back into the backlog

  swarm_m138_free_char(_pruneBuffer);
  delete event;
}
