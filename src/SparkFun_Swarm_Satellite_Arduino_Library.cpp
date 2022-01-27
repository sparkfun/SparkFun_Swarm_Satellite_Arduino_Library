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

  SWARM_M138_error_t err = init();
  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    return true;
  }
  return false;
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

  SWARM_M138_error_t err = init();
  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    return true;
  }
  return false;
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
bool SWARM_M138::begin(byte deviceAddress, TwoWire &wirePort);
{
  if (!initializeBuffers())
    return false;

  _i2cPort = wirePort;
  _address = deviceAddress;

  SWARM_M138_error_t err = init();
  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    return true;
  }
  return false;
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

bool SWARM_M138::bufferedPoll(void)
{
  if (_bufferedPollReentrant == true) // Check for reentry (i.e. bufferedPoll has been called from inside a callback)
    return false;

  _bufferedPollReentrant = true;

  int avail = 0;
  char c = 0;
  bool handled = false;
  unsigned long timeIn = millis();
  char *event;
  int backlogLen = _saraResponseBacklogLength;

  memset(_saraRXBuffer, 0, _RXBuffSize); // Clear _saraRXBuffer

  // Does the backlog contain any data? If it does, copy it into _saraRXBuffer and then clear the backlog
  if (_saraResponseBacklogLength > 0)
  {
    //The backlog also logs reads from other tasks like transmitting.
    if (_printDebug == true)
    {
      _debugPort->print(F("bufferedPoll: backlog found! backlogLen is "));
      _debugPort->println(_saraResponseBacklogLength);
    }
    memcpy(_saraRXBuffer + avail, _saraResponseBacklog, _saraResponseBacklogLength);
    avail += _saraResponseBacklogLength;
    memset(_saraResponseBacklog, 0, _RXBuffSize); // Clear the backlog making sure it is NULL-terminated
    _saraResponseBacklogLength = 0;
  }

  if ((hwAvailable() > 0) || (backlogLen > 0)) // If either new data is available, or backlog had data.
  {
    // Wait for up to _rxWindowMillis for new serial data to arrive. 
    while (((millis() - timeIn) < _rxWindowMillis) && (avail < _RXBuffSize))
    {
      if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        c = readChar();
        // bufferedPoll is only interested in the URCs.
        // The URCs are all readable.
        // strtok does not like NULL characters.
        // So we need to make sure no NULL characters are added to _saraRXBuffer
        if (c == '\0')
          c = '0'; // Convert any NULLs to ASCII Zeros
        _saraRXBuffer[avail++] = c;
        timeIn = millis();
      }
    }

    // _saraRXBuffer now contains the backlog (if any) and the new serial data (if any)

    // A health warning about strtok:
    // strtok will convert any delimiters it finds ("\r\n" in our case) into NULL characters.
    // Also, be very careful that you do not use strtok within an strtok while loop.
    // The next call of strtok(NULL, ...) in the outer loop will use the pointer saved from the inner loop!
    // In our case, strtok is also used in pruneBacklog, which is called by waitForRespone or sendCommandWithResponse,
    // which is called by the parse functions called by processURCEvent...
    // The solution is to use strtok_r - the reentrant version of strtok

    char *preservedEvent;
    event = strtok_r(_saraRXBuffer, "\r\n", &preservedEvent); // Look for an 'event' (_saraRXBuffer contains something ending in \r\n)

    if (event != NULL)
      if (_printDebug == true)
        _debugPort->println(F("bufferedPoll: event(s) found! ===>"));

    while (event != NULL) // Keep going until all events have been processed
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("bufferedPoll: start of event: "));
        _debugPort->println(event);
      }

      //Process the event
      bool latestHandled = processURCEvent((const char *)event);
      if (latestHandled)
        handled = true; // handled will be true if latestHandled has ever been true

      if ((_saraResponseBacklogLength > 0) && ((avail + _saraResponseBacklogLength) < _RXBuffSize)) // Has any new data been added to the backlog?
      {
        if (_printDebug == true)
        {
          _debugPort->println(F("bufferedPoll: new backlog added!"));
        }
        memcpy(_saraRXBuffer + avail, _saraResponseBacklog, _saraResponseBacklogLength);
        avail += _saraResponseBacklogLength;
        memset(_saraResponseBacklog, 0, _RXBuffSize); //Clear out the backlog buffer again.
        _saraResponseBacklogLength = 0;
      }

      //Walk through any remaining events
      event = strtok_r(NULL, "\r\n", &preservedEvent);

      if (_printDebug == true)
        _debugPort->println(F("bufferedPoll: end of event")); //Just to denote end of processing event.

      if (event == NULL)
        if (_printDebug == true)
          _debugPort->println(F("bufferedPoll: <=== end of event(s)!"));
    }
  }

  free(event);

  _bufferedPollReentrant = false;

  return handled;
} // /bufferedPoll

// Parse incoming URC's - the associated parse functions pass the data to the user via the callbacks (if defined)
bool SWARM_M138::processURCEvent(const char *event)
{
  { // URC: +UUSORD (Read Socket Data)
    int socket, length;
    int ret = sscanf(event, "+UUSORD: %d,%d", &socket, &length);
    if (ret == 2)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: read socket data"));
      // From the SWARM_M138 AT Commands Manual:
      // "For the UDP socket type the URC +UUSORD: <socket>,<length> notifies that a UDP packet has been received,
      //  either when buffer is empty or after a UDP packet has been read and one or more packets are stored in the
      //  buffer."
      // So we need to check if this is a TCP socket or a UDP socket:
      //  If UDP, we call parseSocketReadIndicationUDP.
      //  Otherwise, we call parseSocketReadIndication.
      if (_lastSocketProtocol[socket] == SWARM_M138_UDP)
      {
        if (_printDebug == true)
          _debugPort->println(F("processReadEvent: received +UUSORD but socket is UDP. Calling parseSocketReadIndicationUDP"));
        parseSocketReadIndicationUDP(socket, length);
      }
      else
        parseSocketReadIndication(socket, length);
      return true;
    }
  }
  { // URC: +UUSORF (Receive From command (UDP only))
    int socket, length;
    int ret = sscanf(event, "+UUSORF: %d,%d", &socket, &length);
    if (ret == 2)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: UDP receive"));
      parseSocketReadIndicationUDP(socket, length);
      return true;
    }
  }
  { // URC: +UUSOLI (Set Listening Socket)
    int socket = 0;
    int listenSocket = 0;
    unsigned int port = 0;
    unsigned int listenPort = 0;
    IPAddress remoteIP = {0,0,0,0};
    IPAddress localIP = {0,0,0,0};
    int remoteIPstore[4]  = {0,0,0,0};
    int localIPstore[4] = {0,0,0,0};

    int ret = sscanf(event,
                     "+UUSOLI: %d,\"%d.%d.%d.%d\",%u,%d,\"%d.%d.%d.%d\",%u",
                     &socket,
                     &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3],
                     &port, &listenSocket,
                     &localIPstore[0], &localIPstore[1], &localIPstore[2], &localIPstore[3],
                     &listenPort);
    for (int i = 0; i <= 3; i++)
    {
      if (ret >= 5)
        remoteIP[i] = (uint8_t)remoteIPstore[i];
      if (ret >= 11)
        localIP[i] = (uint8_t)localIPstore[i];
    }
    if (ret >= 5)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: socket listen"));
      parseSocketListenIndication(listenSocket, localIP, listenPort, socket, remoteIP, port);
      return true;
    }
  }
  { // URC: +UUSOCL (Close Socket)
    int socket;
    int ret = sscanf(event, "+UUSOCL: %d", &socket);
    if (ret == 1)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: socket close"));
      if ((socket >= 0) && (socket <= 6))
      {
        if (_socketCloseCallback != NULL)
        {
          _socketCloseCallback(socket);
        }
      }
      return true;
    }
  }
  { // URC: +UULOC (Localization information - CellLocate and hybrid positioning)
    ClockData clck;
    PositionData gps;
    SpeedData spd;
    unsigned long uncertainty;
    int scanNum;
    int latH, lonH, alt;
    unsigned int speedU, cogU;
    char latL[10], lonL[10];
    int dateStore[5];

    // Maybe we should also scan for +UUGIND and extract the activated gnss system?

    // This assumes the ULOC response type is "0" or "1" - as selected by gpsRequest detailed
    scanNum = sscanf(event,
                      "+UULOC: %d/%d/%d,%d:%d:%d.%d,%d.%[^,],%d.%[^,],%d,%lu,%u,%u,%*s",
                      &dateStore[0], &dateStore[1], &clck.date.year,
                      &dateStore[2], &dateStore[3], &dateStore[4], &clck.time.ms,
                      &latH, latL, &lonH, lonL, &alt, &uncertainty,
                      &speedU, &cogU);
    clck.date.day = dateStore[0];
    clck.date.month = dateStore[1];
    clck.time.hour = dateStore[2];
    clck.time.minute = dateStore[3];
    clck.time.second = dateStore[4];

    if (scanNum >= 13)
    {
      // Found a Location string!
      if (_printDebug == true)
      {
        _debugPort->println(F("processReadEvent: location"));
      }

      if (latH >= 0)
        gps.lat = (float)latH + ((float)atol(latL) / pow(10, strlen(latL)));
      else
        gps.lat = (float)latH - ((float)atol(latL) / pow(10, strlen(latL)));
      if (lonH >= 0)
        gps.lon = (float)lonH + ((float)atol(lonL) / pow(10, strlen(lonL)));
      else
        gps.lon = (float)lonH - ((float)atol(lonL) / pow(10, strlen(lonL)));
      gps.alt = (float)alt;
      if (scanNum >= 15) // If detailed response, get speed data
      {
        spd.speed = (float)speedU;
        spd.cog = (float)cogU;
      }

      // if (_printDebug == true)
      // {
      //   _debugPort->print(F("processReadEvent: location:  lat: "));
      //   _debugPort->print(gps.lat, 7);
      //   _debugPort->print(F(" lon: "));
      //   _debugPort->print(gps.lon, 7);
      //   _debugPort->print(F(" alt: "));
      //   _debugPort->print(gps.alt, 2);
      //   _debugPort->print(F(" speed: "));
      //   _debugPort->print(spd.speed, 2);
      //   _debugPort->print(F(" cog: "));
      //   _debugPort->println(spd.cog, 2);
      // }

      if (_gpsRequestCallback != NULL)
      {
        _gpsRequestCallback(clck, gps, spd, uncertainty);
      }

      return true;
    }
  }
  { // URC: +UUSIMSTAT (SIM Status)
    SWARM_M138_sim_states_t state;
    int scanNum;
    int stateStore;

    scanNum = sscanf(event, "+UUSIMSTAT:%d", &stateStore); // Note: no space after the colon!

    if (scanNum == 1)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: SIM status"));

      state = (SWARM_M138_sim_states_t)stateStore;

      if (_simStateReportCallback != NULL)
      {
        _simStateReportCallback(state);
      }

      return true;
    }
  }
  { // URC: +UUPSDA (Packet Switched Data Action)
    int result;
    IPAddress remoteIP = {0, 0, 0, 0};
    int scanNum;
    int remoteIPstore[4];

    scanNum = sscanf(event, "+UUPSDA: %d,\"%d.%d.%d.%d\"",
                      &result, &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3]);

    if (scanNum == 5)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: packet switched data action"));

      for (int i = 0; i <= 3; i++)
      {
        remoteIP[i] = (uint8_t)remoteIPstore[i];
      }

      if (_psdActionRequestCallback != NULL)
      {
        _psdActionRequestCallback(result, remoteIP);
      }

      return true;
    }
  }
  { // URC: +UUHTTPCR (HTTP Command Result)
    int profile, command, result;
    int scanNum;

    scanNum = sscanf(event, "+UUHTTPCR: %d,%d,%d", &profile, &command, &result);

    if (scanNum == 3)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: HTTP command result"));

      if ((profile >= 0) && (profile < SWARM_M138_NUM_HTTP_PROFILES))
      {
        if (_httpCommandRequestCallback != NULL)
        {
          _httpCommandRequestCallback(profile, command, result);
        }
      }

      return true;
    }
  }
  { // URC: +UUPING (Ping Result)
    int retry = 0;
    int p_size = 0;
    int ttl = 0;
    String remote_host = "";
    IPAddress remoteIP = {0, 0, 0, 0};
    long rtt = 0;
    int scanNum;
    const char *searchPtr = event;

    // Try to extract the UUPING retries and payload size
    scanNum = sscanf(searchPtr, "+UUPING: %d,%d,", &retry, &p_size);

    if (scanNum == 2)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("processReadEvent: ping"));
      }

      searchPtr = strchr(++searchPtr, '\"'); // Search to the first quote

      // Extract the remote host name, stop at the next quote
      while ((*(++searchPtr) != '\"') && (*searchPtr != '\0'))
      {
        remote_host.concat(*(searchPtr));
      }

      if (*searchPtr != '\0') // Make sure we found a quote
      {
        int remoteIPstore[4];
        scanNum = sscanf(searchPtr, "\",\"%d.%d.%d.%d\",%d,%ld",
                          &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3], &ttl, &rtt);
        for (int i = 0; i <= 3; i++)
        {
          remoteIP[i] = (uint8_t)remoteIPstore[i];
        }

        if (scanNum == 6) // Make sure we extracted enough data
        {
          if (_pingRequestCallback != NULL)
          {
            _pingRequestCallback(retry, p_size, remote_host, remoteIP, ttl, rtt);
          }
        }
      }
      return true;
    }
  }
  return false;
}

void SWARM_M138::setSocketListenCallback(void (*socketListenCallback)(int, IPAddress, unsigned int, int, IPAddress, unsigned int))
{
  _socketListenCallback = socketListenCallback;
}

size_t SWARM_M138::write(uint8_t c)
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
  return (size_t)0;
}

size_t SWARM_M138::write(const char *str)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->print(str);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->print(str);
  }
#endif
  return (size_t)0;
}

size_t SWARM_M138::write(const char *buffer, size_t size)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->write((const uint8_t *)buffer, (int)size);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->write((const uint8_t *)buffer, (int)size);
  }
#endif
  return (size_t)0;
}

String SWARM_M138::getFirmwareVersion(void)
{
  char *response;
  char idResponse[16] = {0x00}; // E.g. 11.40
  SWARM_M138_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SWARM_M138_COMMAND_FW_VER_ID,
                                SWARM_M138_RESPONSE_OK, response, SWARM_M138_STANDARD_RESPONSE_TIMEOUT);
  if (err == SWARM_M138_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", idResponse) != 1)
    {
      memset(idResponse, 0, 16);
    }
  }
  free(response);
  return String(idResponse);
}

String SWARM_M138::clock(void)
{
  SWARM_M138_error_t err;
  char *command;
  char *response;
  char *clockBegin;
  char *clockEnd;

  command = sara_r5_calloc_char(strlen(SWARM_M138_COMMAND_CLOCK) + 2);
  if (command == NULL)
    return "";
  sprintf(command, "%s?", SWARM_M138_COMMAND_CLOCK);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return "";
  }

  err = sendCommandWithResponse(command, SWARM_M138_RESPONSE_OK,
                                response, SWARM_M138_STANDARD_RESPONSE_TIMEOUT);
  if (err != SWARM_M138_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return "";
  }

  // Response format: \r\n+CCLK: "YY/MM/DD,HH:MM:SS-TZ"\r\n\r\nOK\r\n
  clockBegin = strchr(response, '\"'); // Find first quote
  if (clockBegin == NULL)
  {
    free(command);
    free(response);
    return "";
  }
  clockBegin += 1;                     // Increment pointer to begin at first number
  clockEnd = strchr(clockBegin, '\"'); // Find last quote
  if (clockEnd == NULL)
  {
    free(command);
    free(response);
    return "";
  }
  *(clockEnd) = '\0'; // Set last quote to null char -- end string

  String clock = String(clockBegin); // Extract the clock as a String _before_ freeing response

  free(command);
  free(response);

  return (clock);
}

/////////////
// Private //
/////////////

SWARM_M138_error_t SWARM_M138::init(unsigned long baud,
                              SWARM_M138::SWARM_M138_init_type_t initType)
{
  SWARM_M138_error_t err;

  //If we have recursively called init too many times, bail
  _currentInitDepth++;
  if (_currentInitDepth == _maxInitDepth)
  {
    if (_printDebug == true)
      _debugPort->println(F("init: Module failed to init. Exiting."));
    return (SWARM_M138_ERROR_NO_RESPONSE);
  }

  if (_printDebug == true)
    _debugPort->println(F("init: Begin module init."));

  // There's no 'easy' way to tell if the serial port has already been begun for us.
  // We have to assume it has not been begun and so do it here.
  // For special cases like Software Serial on ESP32, we need to begin _and_ end the port externally
  // _before_ calling the SWARM_M138 .begin.
  // See SARA-R5_Example2_Identification_ESPSoftwareSerial for more details.
  beginSerial(baud);

  if (initType == SWARM_M138_INIT_AUTOBAUD)
  {
    if (_printDebug == true)
      _debugPort->println(F("init: Attempting autobaud connection to module."));
    if (autobaud(baud) != SWARM_M138_ERROR_SUCCESS)
    {
      return init(baud, SWARM_M138_INIT_RESET);
    }
  }
  else if (initType == SWARM_M138_INIT_RESET)
  {
    if (_printDebug == true)
      _debugPort->println(F("init: Power cycling module."));
    powerOff();
    delay(SWARM_M138_POWER_OFF_PULSE_PERIOD);
    powerOn();
    delay(2000);
    if (at() != SWARM_M138_ERROR_SUCCESS)
    {
      return init(baud, SWARM_M138_INIT_AUTOBAUD);
    }
  }

  // Use disable echo to test response
  err = enableEcho(false);

  if (err != SWARM_M138_ERROR_SUCCESS)
  {
    if (_printDebug == true)
      _debugPort->println(F("init: Module failed echo test."));
    return init(baud, SWARM_M138_INIT_AUTOBAUD);
  }

  if (_printDebug == true)
    _debugPort->println(F("init: Module responded successfully."));

  _baud = baud;
  setGpioMode(GPIO1, NETWORK_STATUS);
  //setGpioMode(GPIO2, GNSS_SUPPLY_ENABLE);
  setGpioMode(GPIO6, TIME_PULSE_OUTPUT);
  setSMSMessageFormat(SWARM_M138_MESSAGE_FORMAT_TEXT);
  autoTimeZone(_autoTimeZoneForBegin);
  for (int i = 0; i < SWARM_M138_NUM_SOCKETS; i++)
  {
    socketClose(i, 100);
  }

  return SWARM_M138_ERROR_SUCCESS;
}

SWARM_M138_error_t SWARM_M138::waitForResponse(const char *expectedResponse, const char *expectedError, uint16_t timeout)
{
  unsigned long timeIn;
  bool found = false;
  int responseIndex = 0, errorIndex = 0;
  // bool printedSomething = false;

  timeIn = millis();

  while ((!found) && ((timeIn + timeout) > millis()))
  {
    if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      char c = readChar();
      // if (_printDebug == true)
      // {
      //   if (printedSomething == false)
      //     _debugPort->print(F("waitForResponse: "));
      //   _debugPort->write(c);
      //   printedSomething = true;
      // }
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
      //_saraResponseBacklog is a global array that holds the backlog of any events
      //that came in while waiting for response. To be processed later within bufferedPoll().
      //Note: the expectedResponse or expectedError will also be added to the backlog.
      //The backlog is only used by bufferedPoll to process the URCs - which are all readable.
      //bufferedPoll uses strtok - which does not like NULL characters.
      //So let's make sure no NULLs end up in the backlog!
      if (_saraResponseBacklogLength < _RXBuffSize) // Don't overflow the buffer
      {
        if (c == '\0')
          _saraResponseBacklog[_saraResponseBacklogLength++] = '0'; // Change NULLs to ASCII Zeros
        else
          _saraResponseBacklog[_saraResponseBacklogLength++] = c;
      }
    }
  }

  // if (_printDebug == true)
  //   if (printedSomething)
  //     _debugPort->println();

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

  return SWARM_M138_ERROR_NO_RESPONSE;
}

SWARM_M138_error_t SWARM_M138::sendCommandWithResponse(
    const char *command, const char *expectedResponse, char *responseDest,
    unsigned long commandTimeout, int destSize, bool at)
{
  bool found = false;
  int index = 0;
  int destIndex = 0;
  unsigned int charsRead = 0;
  //bool printedSomething = false;

  if (_printDebug == true)
  {
    _debugPort->print(F("sendCommandWithResponse: Command: "));
    _debugPort->println(String(command));
  }

  sendCommand(command, at); //Sending command needs to dump data to backlog buffer as well.
  unsigned long timeIn = millis();

  while ((!found) && ((timeIn + commandTimeout) > millis()))
  {
    if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      char c = readChar();
      // if (_printDebug == true)
      // {
      //   if (printedSomething == false)
      //   {
      //     _debugPort->print(F("sendCommandWithResponse: Response: "));
      //     printedSomething = true;
      //   }
      //   _debugPort->write(c);
      // }
      if (responseDest != NULL)
      {
        if (destIndex < destSize) // Only add this char to response if there is room for it
          responseDest[destIndex] = c;
        destIndex++;
        if (destIndex == destSize)
        {
          if (_printDebug == true)
          {
            // if (printedSomething)
            //   _debugPort->println();
            _debugPort->print(F("sendCommandWithResponse: Panic! responseDest is full!"));
            // if (printedSomething)
            //   _debugPort->print(F("sendCommandWithResponse: Ignored response: "));
          }
        }
      }
      charsRead++;
      if (c == expectedResponse[index])
      {
        if (++index == (int)strlen(expectedResponse))
        {
          found = true;
        }
      }
      else
      {
        index = 0;
      }
      //_saraResponseBacklog is a global array that holds the backlog of any events
      //that came in while waiting for response. To be processed later within bufferedPoll().
      //Note: the expectedResponse or expectedError will also be added to the backlog
      //The backlog is only used by bufferedPoll to process the URCs - which are all readable.
      //bufferedPoll uses strtok - which does not like NULL characters.
      //So let's make sure no NULLs end up in the backlog!
      if (_saraResponseBacklogLength < _RXBuffSize) // Don't overflow the buffer
      {
        if (c == '\0')
          _saraResponseBacklog[_saraResponseBacklogLength++] = '0'; // Change NULLs to ASCII Zeros
        else
          _saraResponseBacklog[_saraResponseBacklogLength++] = c;
      }
    }
  }

  // if (_printDebug == true)
  //   if (printedSomething)
  //     _debugPort->println();

  pruneBacklog(); // Prune any incoming non-actionable URC's and responses/errors from the backlog

  if (found)
  {
    return SWARM_M138_ERROR_SUCCESS;
  }
  else if (charsRead == 0)
  {
    return SWARM_M138_ERROR_NO_RESPONSE;
  }
  else
  {
    return SWARM_M138_ERROR_UNEXPECTED_RESPONSE;
  }
}

// Send a custom command with an expected (potentially partial) response, store entire response
SWARM_M138_error_t SWARM_M138::sendCustomCommandWithResponse(const char *command, const char *expectedResponse,
                                                       char *responseDest, unsigned long commandTimeout, bool at)
{
  // Assume the user has allocated enough storage for any response. Set destSize to 32766.
  return sendCommandWithResponse(command, expectedResponse, responseDest, commandTimeout, 32766, at);
}

void SWARM_M138::sendCommand(const char *command, bool at)
{
  //Spend up to _rxWindowMillis milliseconds copying any incoming serial data into the backlog
  unsigned long timeIn = millis();
  if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
  {
    while (((millis() - timeIn) < _rxWindowMillis) && (_saraResponseBacklogLength < _RXBuffSize)) //May need to escape on newline?
    {
      if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        //_saraResponseBacklog is a global array that holds the backlog of any events
        //that came in while waiting for response. To be processed later within bufferedPoll().
        //Note: the expectedResponse or expectedError will also be added to the backlog
        //The backlog is only used by bufferedPoll to process the URCs - which are all readable.
        //bufferedPoll uses strtok - which does not like NULL characters.
        //So let's make sure no NULLs end up in the backlog!
        char c = readChar();
        if (c == '\0') // Make sure no NULL characters end up in the backlog! Change them to ASCII Zeros
          c = '0';
        _saraResponseBacklog[_saraResponseBacklogLength++] = c;
        timeIn = millis();
      }
    }
  }

  //Now send the command
  if (at)
  {
    hwPrint(SWARM_M138_COMMAND_AT);
    hwPrint(command);
    hwPrint("\r");
  }
  else
  {
    hwPrint(command);
  }
}

SWARM_M138_error_t SWARM_M138::parseSocketReadIndication(int socket, int length)
{
  SWARM_M138_error_t err;
  char *readDest;

  if ((socket < 0) || (length < 0))
  {
    return SWARM_M138_ERROR_UNEXPECTED_RESPONSE;
  }

  // Return now if both callbacks pointers are NULL - otherwise the data will be read and lost!
  if ((_socketReadCallback == NULL) && (_socketReadCallbackPlus == NULL))
    return SWARM_M138_ERROR_INVALID;

  readDest = sara_r5_calloc_char(length + 1);
  if (readDest == NULL)
    return SWARM_M138_ERROR_OUT_OF_MEMORY;

  int bytesRead;
  err = socketRead(socket, length, readDest, &bytesRead);
  if (err != SWARM_M138_ERROR_SUCCESS)
  {
    free(readDest);
    return err;
  }

  if (_socketReadCallback != NULL)
  {
    String dataAsString = ""; // Create an empty string
    // Copy the data from readDest into the String in a binary-compatible way
    // Important Note: some implementations of concat, like the one on ESP32, are binary-compatible.
    // But some, like SAMD, are not. They use strlen or strcpy internally - which don't like \0's.
    // The only true binary-compatible solution is to use socketReadCallbackPlus...
    for (int i = 0; i < bytesRead; i++)
      dataAsString.concat(readDest[i]);
    _socketReadCallback(socket, dataAsString);
  }

  if (_socketReadCallbackPlus != NULL)
  {
    IPAddress dummyAddress = { 0, 0, 0, 0 };
    int dummyPort = 0;
    _socketReadCallbackPlus(socket, (const char *)readDest, bytesRead, dummyAddress, dummyPort);
  }

  free(readDest);
  return SWARM_M138_ERROR_SUCCESS;
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

  return (size_t)0;
}

int SWARM_M138::readAvailable(char *inString)
{
  int len = 0;

  if (_hardSerial != NULL)
  {
    while (_hardSerial->available())
    {
      char c = (char)_hardSerial->read();
      if (inString != NULL)
      {
        inString[len++] = c;
      }
    }
    if (inString != NULL)
    {
      inString[len] = 0;
    }
    //if (_printDebug == true)
    //  _debugPort->println(inString);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    while (_softSerial->available())
    {
      char c = (char)_softSerial->read();
      if (inString != NULL)
      {
        inString[len++] = c;
      }
    }
    if (inString != NULL)
    {
      inString[len] = 0;
    }
  }
#endif

  return len;
}

char SWARM_M138::readChar(void)
{
  char ret = 0;

  if (_hardSerial != NULL)
  {
    ret = (char)_hardSerial->read();
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    ret = (char)_softSerial->read();
  }
#endif

  return ret;
}

int SWARM_M138::hwAvailable(void)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->available();
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->available();
  }
#endif

  return -1;
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

void SWARM_M138::setTimeout(unsigned long timeout)
{
  if (_hardSerial != NULL)
  {
    _hardSerial->setTimeout(timeout);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    _softSerial->setTimeout(timeout);
  }
#endif
}

bool SWARM_M138::find(char *target)
{
  bool found = false;
  if (_hardSerial != NULL)
  {
    found = _hardSerial->find(target);
  }
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    found = _softSerial->find(target);
  }
#endif
  return found;
}

char *SWARM_M138::sara_r5_calloc_char(size_t num)
{
  return (char *)calloc(num, sizeof(char));
}

//This prunes the backlog of non-actionable events. If new actionable events are added, you must modify the if statement.
void SWARM_M138::pruneBacklog()
{
  char *event;

  // if (_printDebug == true)
  // {
  //   if (_saraResponseBacklogLength > 0) //Handy for debugging new parsing.
  //   {
  //     _debugPort->println(F("pruneBacklog: before pruning, backlog was:"));
  //     _debugPort->println(_saraResponseBacklog);
  //     _debugPort->println(F("pruneBacklog: end of backlog"));
  //   }
  //   else
  //   {
  //     _debugPort->println(F("pruneBacklog: backlog was empty"));
  //   }
  // }

  memset(_pruneBuffer, 0, _RXBuffSize); // Clear the _pruneBuffer

  _saraResponseBacklogLength = 0; // Zero the backlog length

  char *preservedEvent;
  event = strtok_r(_saraResponseBacklog, "\r\n", &preservedEvent); // Look for an 'event' - something ending in \r\n

  while (event != NULL) //If event is actionable, add it to pruneBuffer.
  {
    // These are the events we want to keep so they can be processed by poll / bufferedPoll
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
      strcat(_pruneBuffer, "\r\n"); // strtok blows away delimiter, but we want that for later.
      _saraResponseBacklogLength += strlen(event) + 2; // Add the length of this event to _saraResponseBacklogLength
    }

    event = strtok_r(NULL, "\r\n", &preservedEvent); // Walk though any remaining events
  }

  memset(_saraResponseBacklog, 0, _RXBuffSize); //Clear out backlog buffer.
  memcpy(_saraResponseBacklog, _pruneBuffer, _saraResponseBacklogLength); //Copy the pruned buffer back into the backlog

  // if (_printDebug == true)
  // {
  //   if (_saraResponseBacklogLength > 0) //Handy for debugging new parsing.
  //   {
  //     _debugPort->println(F("pruneBacklog: after pruning, backlog is now:"));
  //     _debugPort->println(_saraResponseBacklog);
  //     _debugPort->println(F("pruneBacklog: end of backlog"));
  //   }
  //   else
  //   {
  //     _debugPort->println(F("pruneBacklog: backlog is now empty"));
  //   }
  // }

  free(event);
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
const char *modemErrorString(Swarm_M138_Error_e error)
{
  switch (error)
  {
    case SWARM_M138_ERROR_ERROR:
      return "Just a plain old communication error";
      break;
    case SWARM_M138_ERROR_SUCCESS:
      return "Hey, it worked!";
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

// GPS Helper Functions:

// Read a source string until a delimiter is hit, store the result in destination
char *SWARM_M138::readDataUntil(char *destination, unsigned int destSize,
                             char *source, char delimiter)
{

  char *strEnd;
  size_t len;

  strEnd = strchr(source, delimiter);

  if (strEnd != NULL)
  {
    len = strEnd - source;
    memset(destination, 0, destSize);
    memcpy(destination, source, len);
  }

  return strEnd;
}

bool SWARM_M138::parseGPRMCString(char *rmcString, PositionData *pos,
                               ClockData *clk, SpeedData *spd)
{
  char *ptr, *search;
  unsigned long tTemp;
  char tempData[TEMP_NMEA_DATA_SIZE];

  // if (_printDebug == true)
  // {
  //   _debugPort->println(F("parseGPRMCString: rmcString: "));
  //   _debugPort->println(rmcString);
  // }

  // Fast-forward test to first value:
  ptr = strchr(rmcString, ',');
  ptr++; // Move ptr past first comma

  // If the next character is another comma, there's no time data
  // Find time:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  // Next comma should be present and not the next position
  if ((search != NULL) && (search != ptr))
  {
    pos->utc = atof(tempData);                             // Extract hhmmss.ss as float
    tTemp = pos->utc;                                      // Convert to unsigned long (discard the digits beyond the decimal point)
    clk->time.ms = ((unsigned int)(pos->utc * 100)) % 100; // Extract the milliseconds
    clk->time.hour = tTemp / 10000;
    tTemp -= ((unsigned long)clk->time.hour * 10000);
    clk->time.minute = tTemp / 100;
    tTemp -= ((unsigned long)clk->time.minute * 100);
    clk->time.second = tTemp;
  }
  else
  {
    pos->utc = 0.0;
    clk->time.hour = 0;
    clk->time.minute = 0;
    clk->time.second = 0;
  }
  ptr = search + 1; // Move pointer to next value

  // Find status character:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  // Should be a single character: V = Data invalid, A = Data valid
  if ((search != NULL) && (search == ptr + 1))
  {
    pos->status = *ptr; // Assign char at ptr to status
  }
  else
  {
    pos->status = 'X'; // Made up very bad status
  }
  ptr = search + 1;

  // Find latitude:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    pos->lat = atof(tempData);              // Extract ddmm.mmmmm as float
    unsigned long lat_deg = pos->lat / 100; // Extract the degrees
    pos->lat -= (float)lat_deg * 100.0;     // Subtract the degrees leaving only the minutes
    pos->lat /= 60.0;                       // Convert minutes into degrees
    pos->lat += (float)lat_deg;             // Finally add the degrees back on again
  }
  else
  {
    pos->lat = 0.0;
  }
  ptr = search + 1;

  // Find latitude hemishpere
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search == ptr + 1))
  {
    if (*ptr == 'S')    // Is the latitude South
      pos->lat *= -1.0; // Make lat negative
  }
  ptr = search + 1;

  // Find longitude:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    pos->lon = atof(tempData);              // Extract dddmm.mmmmm as float
    unsigned long lon_deg = pos->lon / 100; // Extract the degrees
    pos->lon -= (float)lon_deg * 100.0;     // Subtract the degrees leaving only the minutes
    pos->lon /= 60.0;                       // Convert minutes into degrees
    pos->lon += (float)lon_deg;             // Finally add the degrees back on again
  }
  else
  {
    pos->lon = 0.0;
  }
  ptr = search + 1;

  // Find longitude hemishpere
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search == ptr + 1))
  {
    if (*ptr == 'W')    // Is the longitude West
      pos->lon *= -1.0; // Make lon negative
  }
  ptr = search + 1;

  // Find speed
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    spd->speed = atof(tempData); // Extract speed over ground in knots
    spd->speed *= 0.514444;      // Convert to m/s
  }
  else
  {
    spd->speed = 0.0;
  }
  ptr = search + 1;

  // Find course over ground
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    spd->cog = atof(tempData);
  }
  else
  {
    spd->cog = 0.0;
  }
  ptr = search + 1;

  // Find date
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    tTemp = atol(tempData);
    clk->date.day = tTemp / 10000;
    tTemp -= ((unsigned long)clk->date.day * 10000);
    clk->date.month = tTemp / 100;
    tTemp -= ((unsigned long)clk->date.month * 100);
    clk->date.year = tTemp;
  }
  else
  {
    clk->date.day = 0;
    clk->date.month = 0;
    clk->date.year = 0;
  }
  ptr = search + 1;

  // Find magnetic variation in degrees:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    spd->magVar = atof(tempData);
  }
  else
  {
    spd->magVar = 0.0;
  }
  ptr = search + 1;

  // Find magnetic variation direction
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search == ptr + 1))
  {
    if (*ptr == 'W')       // Is the magnetic variation West
      spd->magVar *= -1.0; // Make magnetic variation negative
  }
  ptr = search + 1;

  // Find position system mode
  // Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, A = Autonomous GNSS fix,
  //                              D = Differential GNSS fix, F = RTK float, R = RTK fixed
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, '*');
  if ((search != NULL) && (search = ptr + 1))
  {
    pos->mode = *ptr;
  }
  else
  {
    pos->mode = 'X';
  }
  ptr = search + 1;

  if (pos->status == 'A')
  {
    return true;
  }
  return false;
}
