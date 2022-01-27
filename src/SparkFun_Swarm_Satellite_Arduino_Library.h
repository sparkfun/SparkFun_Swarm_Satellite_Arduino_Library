/*!
 * @file SparkFun_Swarm_Satellite_Arduino_Library.h
 *
 * SparkFun Swarm Satellite Arduino Library
 * 
 * This library facilitates communication with the Swarm M138 satellite modem.
 * 
 * Want to support open source hardware? Buy a board from SparkFun!
 * <br>SparkX Swarm Serial Breakout (SPX-19236): https://www.sparkfun.com/products/19236
 * 
 * This library was written by:
 * Paul Clark
 * SparkFun Electronics
 * January 2022
 * 
 * Please see LICENSE.md for the license information
 * 
 */

#ifndef SPARKFUN_SWARM_SATELLITE_ARDUINO_LIBRARY_H
#define SPARKFUN_SWARM_SATELLITE_ARDUINO_LIBRARY_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef ARDUINO_ARCH_AVR                     // Arduino AVR boards (Uno, Pro Micro, etc.)
#define SWARM_M138_SOFTWARE_SERIAL_ENABLED  // Enable software serial
#endif

#ifdef ARDUINO_ARCH_SAMD                    // Arduino SAMD boards (SAMD21, etc.)
#define SWARM_M138_SOFTWARE_SERIAL_ENABLEDx // Disable software serial
#endif

#ifdef ARDUINO_ARCH_APOLLO3                 // Arduino Apollo boards (Artemis module, RedBoard Artemis, etc)
#define SWARM_M138_SOFTWARE_SERIAL_ENABLEDx // Disable software serial (no longer supported with v2 of Apollo3)
// Note: paulvha has provided software serial support for v2 of the Apollo3 / Artemis core.
//       Further details are available at:
//       https://github.com/paulvha/apollo3/tree/master/SoftwareSerial
#endif

#ifdef ARDUINO_ARCH_STM32                   // STM32 based boards (Disco, Nucleo, etc)
#define SWARM_M138_SOFTWARE_SERIAL_ENABLED  // Enable software serial
#endif

#ifdef ARDUINO_ARCH_ESP32 // ESP32 based boards
// Check to see if ESP Software Serial has been included
// Note: you need to #include <SoftwareSerial.h> at the very start of your script,
// _before_ the #include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>, for this to work.
#if __has_include(<SoftwareSerial.h> )
#define SWARM_M138_SOFTWARE_SERIAL_ENABLED // Enable software serial
#else
#define SWARM_M138_SOFTWARE_SERIAL_ENABLEDx // Disable software serial
#endif
#endif

#ifdef ARDUINO_ARCH_ESP8266 // ESP8266 based boards
// Check to see if ESP Software Serial has been included
// Note: you need to #include <SoftwareSerial.h> at the very start of your script,
// _before_ the #include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>, for this to work.
#if __has_include(<SoftwareSerial.h> )
#define SWARM_M138_SOFTWARE_SERIAL_ENABLED // Enable software serial
#else
#define SWARM_M138_SOFTWARE_SERIAL_ENABLEDx // Disable software serial
#endif
#endif

#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
#include <SoftwareSerial.h> // SoftwareSerial.h is guarded. It is OK to include it twice.
#endif

/** The number of the digital pin connected to GPIO1 can be passed to begin */
#define SWARM_M138_GPIO1_PIN -1 ///< _GPIO1Pin will default to no pin

/** Timeouts for the serial commands */
#define SWARM_M138_STANDARD_RESPONSE_TIMEOUT 1000 ///< Standard command timeout: allow one second for the modem to respond

/** Modem Serial Baud Rate */
#define SWARM_M138_SERIAL_BAUD_RATE 115200 ///< The modem serial baud rate is 115200 and cannot be changed

/** Default I2C address used by the Qwiic Swarm Breakout. Can be changed. */
#define SFE_QWIIC_SWARM_DEFAULT_I2C_ADDRESS 0x52 ///< The default I2C address for the SparkFun Qwiic Swarm Breakout

/** Swarm packet length */
#define SWARM_M138_MAX_PACKET_LENGTH_BYTES 192 ///< The maximum packet length - defined as binary bytes
#define SWARM_M138_MAX_PACKET_LENGTH_HEX 384 ///< The maximum packet length - encoded as ASCII Hex

/** Suported Commands */
const char SWARM_M138_COMMAND_CONFIGURATION[] = "$CS";    ///< Configuration Settings
const char SWARM_M138_COMMAND_DATE_TIME_STAT[] = "$DT";   ///< Date/Time Status
const char SWARM_M138_COMMAND_FIRMWARE_VER[] = "$FV";     ///< Retrieve Firmware Version
const char SWARM_M138_COMMAND_GPS_JAMMING[] = "$GJ";      ///< GPS Jamming/Spoofing Indication
const char SWARM_M138_COMMAND_GEOSPATIAL_INFO[] = "$GN";  ///< Geospatial Information
const char SWARM_M138_COMMAND_GPIO1_CONTROL[] = "$GP";    ///< GPIO1 Control
const char SWARM_M138_COMMAND_GPS_FIX_QUAL[] = "$GS";     ///< GPS Fix Quality
const char SWARM_M138_COMMAND_MSG_RX_MGMT[] = "$MM";      ///< Messages Received Management
const char SWARM_M138_COMMAND_MSG_TX_MGMT[] = "$MT";      ///< Messages to Transmit Management
const char SWARM_M138_COMMAND_POWER_OFF[] = "$PO";        ///< Power Off
const char SWARM_M138_COMMAND_POWER_STAT[] = "$PW";       ///< Power Status
const char SWARM_M138_COMMAND_RX_DATA_MSG[] = "$RD";      ///< Receive Data Message
const char SWARM_M138_COMMAND_RESTART [] = "$RS";         ///< Restart Device
const char SWARM_M138_COMMAND_RX_TEST[] = "$RT";          ///< Receive Test
const char SWARM_M138_COMMAND_SLEEP[] = "$SL";            ///< Sleep Mode
const char SWARM_M138_COMMAND_MODEM_STAT[] = "$M138";     ///< Modem Status
const char SWARM_M138_COMMAND_TX_DATA[] = "$TD";          ///< Transmit Data

/** An enum defining the command result */
enum SWARM_M138_error_t
{
  SWARM_M138_ERROR_ERROR = 0,         ///< Just a plain old communication error
  SWARM_M138_ERROR_SUCCESS,           ///< Hey, it worked!
  SWARM_M138_ERROR_TIMEOUT,           ///< Communication timeout
  SWARM_M138_ERROR_INVALID_CHECKSUM,  ///< Indicates the command response checksum was invalid
  SWARM_M138_ERROR_ERR,               ///< Command input error (ERR)
  SWARM_M138_ERROR_MM_BADPARAM,       ///< Messages Received Management : invalid command or argument
  SWARM_M138_ERROR_MM_DBXINVMSGID,    ///< Messages Received Management : invalid message ID
  SWARM_M138_ERROR_MM_DBXNOMORE,      ///< Messages Received Management : no messages found
  SWARM_M138_ERROR_MT_BADPARAM,       ///< Messages To Transmit Management : invalid command or argument
  SWARM_M138_ERROR_MT_DBXINVMSGID,    ///< Messages To Transmit Management : invalid message ID
  SWARM_M138_ERROR_MT_DBXNOMORE,      ///< Messages To Transmit Management : no messages found
  SWARM_M138_ERROR_SL_TIMENOTSET,     ///< Sleep Mode : time not yet set from GPS
  SWARM_M138_ERROR_SL_BADPARAM,       ///< Sleep Mode : invalid seconds / dateTime
  SWARM_M138_ERROR_SL_NOCOMMAND,      ///< Sleep Mode : No S or U partameter
  SWARM_M138_ERROR_SL_NOTIME,         ///< Sleep Mode : attempt to sleep before time is set
};
#define SWARM_M138_SUCCESS SWARM_M138_ERROR_SUCCESS ///< Hey, it worked!

/** A struct to hold the date and time returned by $DT */
struct Swarm_M138_DateTimeData_t
{
  uint16_t YYYY;  // Year:    1970..2099
  uint8_t MM;     // Month:   01..12
  uint8_t DD;     // Day:     01..31
  uint8_t hh;     // Hour:    00..23
  uint8_t mm;     // Minutes: 00..59
  uint8_t ss;     // Seconds: 00..59
  bool valid;     // flag: I == Invalid == false; V == Valid == true
};

/** A struct to hold the geospatial data returned by $GN */
struct Swarm_M138_GeospatialData_t
{
  float lat; // Degrees: +/- 90
  float lon; // Degrees: +/- 180
  float alt; // m
  float course; // Degrees: 0..359 : 0=north, 90=east, 180=south, and 270=west
  float speed;  // km/h
};

/** A struct to hold the GPS jamming / spoofing indication */
struct Swarm_M138_GPS_Jamming_Indication_t
{
  uint8_t spoof_state;  // 0 Spoofing unknown or deactivated
                        // 1 No spoofing indicated
                        // 2 Spoofing indicated
                        // 3 Multiple spoofing indications
  uint8_t jamming_level;  // 0 = no CW jamming, 255 = strong CW jamming
};

/** Enum for the GPIO1 pin modes */
enum Swarm_M138_GPIO1_Mode_e
{
  SWARM_M138_GPIO1_ANALOG = 0,
  SWARM_M138_GPIO1_EXIT_SLEEP_LOW_HIGH,
  SWARM_M138_GPIO1_EXIT_SLEEP_HIGH_LOW,
  SWARM_M138_GPIO1_OUTPUT_LOW,
  SWARM_M138_GPIO1_OUTPUT_HIGH,
  SWARM_M138_GPIO1_MESSAGES_PENDING_LOW,
  SWARM_M138_GPIO1_MESSAGES_PENDING_HIGH,
  SWARM_M138_GPIO1_SLEEP_MODE_LOW,
  SWARM_M138_GPIO1_SLEEP_MODE_HIGH
};

/** Enum for the GPS fix type */
enum Swarm_M138_GPS_Fix_Type_t
{
  SWARM_M138_GPS_FIX_TYPE_NF = 0, // No Fix
  SWARM_M138_GPS_FIX_TYPE_DR, // Dead reckoning only solution
  SWARM_M138_GPS_FIX_TYPE_G2, // Standalone 2D solution
  SWARM_M138_GPS_FIX_TYPE_G3, // Standalone 3D solution
  SWARM_M138_GPS_FIX_TYPE_D2, // Differential 2D solution
  SWARM_M138_GPS_FIX_TYPE_D3, // Differential 3D solution
  SWARM_M138_GPS_FIX_TYPE_RK, // Combined GNSS + dead reckoning solution
  SWARM_M138_GPS_FIX_TYPE_TT  // Time only solution
}

/** A struct to hold the GPS fix quality */
struct Swarm_M138_GPS_Fix_Quality_t
{
  uint16_t hdop; // Horizontal dilution of precision (0..9999) (integer = actual hdop * 100)
  uint16_t vdop; //  Vertical dilution of precision (0..9999) (integer = actual vdop * 100)
  uint8_t gnss_sats; // Number of GNSS satellites used in solution (integer)
  uint8_t unused; // Always reads as 0, unused
  Swarm_M138_GPS_Fix_Type_t fix_type;
}

/** A struct to hold the power staus info */
struct Swarm_M138_Power_Status_t
{
  float unused1;
  float unused2;
  float unused3;
  float unused4;
  float temp; // CPU Temperature in degrees C to one decimal point
}

/** A struct to hold the receive test results */
struct Swarm_M138_Receive_Test_t
{
  int16_t rssi_background; // Received background noise signal strength in dBm for open channel (integer).
  int16_t rssi_sat; // Received signal strength in dBm for packet from satellite (integer)
  int16_t snr; // Signal to noise ratio in dB for packet (integer)
  int16_t fdev; // Frequency deviation in Hz for packet (integer)
  Swarm_M138_DateTimeData_t time; // Date and time (UTC) of received packet (valid flag is unused - always set to true)
  uint32_t sat_id; // Device ID of satellite heard (hexadecimal)
}

/** Communication interface for the Swarm M138 satellite modem. Inherits the Print Class. */
class SWARM_M138 : public Print
{
public:
  // Constructor
  // The library will use the GPIO1 pin (if provided) to (e.g.) exit sleep mode or indicate pending messages
  /** @brief Class to communicate with the Swarm M138 satellite modem */
  SWARM_M138(int gpio1Pin = SWARM_M138_GPIO1_PIN);

  /** Begin -- initialize module and ensure it's connected */
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  bool begin(SoftwareSerial &softSerial);
#endif
  bool begin(HardwareSerial &hardSerial);
  bool begin(byte deviceAddress = SFE_QWIIC_SWARM_DEFAULT_I2C_ADDRESS, TwoWire &wirePort = Wire);

  /** Debug prints */
  void enableDebugging(Stream &debugPort = Serial); //Turn on debug printing. If user doesn't specify then Serial will be used.

  /** Commands */

  /** Configuration Settings */
  SWARM_M138_error_t getConfigurationSettings(char *settings); // Get the Swarm device ID and type name
  SWARM_M138_error_t getDeviceID(unit32_t *id); // Get the Swarm device ID

  /** Date/Time */
  SWARM_M138_error_t getDateTime(Swarm_M138_DateTimeData_t *dateTime); // Get the most recent $DT message
  SWARM_M138_error_t getDateTimeRate(uint32_t *rate); // Query the current $DT rate
  SWARM_M138_error_t setDateTimeRate(uint32_t rate); // Set the rate of $DT messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Firmware Version */
  SWARM_M138_error_t getFirmwareVersion(char *version); // Get the Swarm device firmware version

  /** GPS Jamming/Spoofing Indication */
  SWARM_M138_error_t getGpsJammingIndication(Swarm_M138_GPS_Jamming_Indication_t *jamming); // Get the most recent $GJ message
  SWARM_M138_error_t getGpsJammingIndicationRate(uint32_t *rate); // Query the current $GJ rate
  SWARM_M138_error_t setGpsJammingIndicationRate(uint32_t rate); // Set the rate of $GJ messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Geospatial information */
  SWARM_M138_error_t getGeospatialInfo(Swarm_M138_GeospatialData_t *info); // Get the most recent $GN message
  SWARM_M138_error_t getGeospatialInfoRate(uint32_t *rate); // Query the current $GN rate
  SWARM_M138_error_t setGeospatialInfoRate(uint32_t rate); // Set the rate of $GN messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** GPIO1 Control */
  SWARM_M138_error_t getGPIO1Mode(Swarm_M138_GPIO1_Mode_e *mode); // Get the GPIO1 pin mode
  SWARM_M138_error_t setGPIO1Mode(Swarm_M138_GPIO1_Mode_e mode); // Set the GPIO1 pin mode

  /** GPS fix quality */
  SWARM_M138_error_t getGpsFixQuality(Swarm_M138_GPS_Fix_Quality_t *fixQuality); // Get the most recent $GS message
  SWARM_M138_error_t getGpsFixQualityRate(uint32_t *rate); // Query the current $GS rate
  SWARM_M138_error_t setGpsFixQualityRate(uint32_t rate); // Set the rate of $GS messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Power Off */
  SWARM_M138_error_t powerOff(void); // The Modem enters a low power mode until power is completely removed and restored

  /** Power Status */
  SWARM_M138_error_t getPowerStatus(Swarm_M138_Power_Status_t *powerStatus); // Get the most recent $PW message
  SWARM_M138_error_t getPowerStatusRate(uint32_t *rate); // Query the current $PW rate
  SWARM_M138_error_t setPowerStatusRate(uint32_t rate); // Set the rate of $PW messages. 0 == Disable. Max is 2147483647 (2^31 - 1)
  SWARM_M138_error_t getTemperature(float *temperature); // Get the most recent temperature

  /** Restart Device */
  SWARM_M138_error_t restartDevice(void); // Restart the modem

  /** Receive Test */
  SWARM_M138_error_t getReceiveTest(Swarm_M138_Receive_Test_t *rxTest); // Get the most recent $RT message
  SWARM_M138_error_t getReceiveTestRate(uint32_t *rate); // Query the current $RT rate
  SWARM_M138_error_t setReceiveTestRate(uint32_t rate); // Set the rate of $RT messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Sleep Mode */
  SWARM_M138_error_t sleepMode(uint32_t seconds); // Sleep for this many seconds
  SWARM_M138_error_t sleepMode(Swarm_M138_DateTimeData_t sleepUntil); // Sleep until this date and time

  /** Messages Received Management */
  SWARM_M138_error_t getRxMessageCount(uint16_t *count, bool unread = false); // Return count of all messages (default) or unread messages (unread = true)
  SWARM_M138_error_t deleteRxMessage(uint64_t id); // Delete RX message with ID
  SWARM_M138_error_t deleteAllRxMessages(bool read = true); // Delete all read RX messages (default) or all messages (read = false)
  SWARM_M138_error_t markRxMessage(uint64_t id); // Mark message ID as read
  SWARM_M138_error_t markAllRxMessages(void); // Mark all messages as read
  SWARM_M138_error_t getMessageNotifications(bool *enabled); // Query if message notifications are enabled
  SWARM_M138_error_t setMessageNotifications(bool enable); // Enable / disable message notifications
  SWARM_M138_error_t readMessage(uint64_t id, char *asciiHex, uint32_t *epoch = NULL, uint16_t *appID = NULL); // Read the message with ID. Message contents are copied to asciiHex as ASCII Hex
  SWARM_M138_error_t readOldestMessage(char *asciiHex, uint64_t *id, uint32_t *epoch = NULL, uint16_t *appID = NULL); // Read the oldest message. Message contents are copied to asciiHex. ID is copied to id.
  SWARM_M138_error_t readNewestMessage(char *asciiHex, uint64_t *id, uint32_t *epoch = NULL, uint16_t *appID = NULL); // Read the oldest message. Message contents are copied to asciiHex. ID is copied to id.

  /** Messages To Transmit Management */
  SWARM_M138_error_t getUnsentMessageCount(uint16_t *count); // Return count of all unsent messages
  SWARM_M138_error_t deleteTxMessage(uint64_t id); // Delete TX message with ID
  SWARM_M138_error_t deleteAllTxMessages(void); // Delete all unsent messages
  SWARM_M138_error_t listTxMessage(uint64_t id, char *asciiHex, uint32_t *epoch = NULL); // List unsent message with ID
  SWARM_M138_error_t listTxMessagesIDs(uint64_t[] *ids); // List the IDs of all unsent messages. Call getUnsentMessageCount first so you know how many IDs to expect

  /** Transmit Data */
  // The application ID is optional. Valid appID's are: 0 to 64999. Swarm reserves use of 65000 - 65535.
  SWARM_M138_error_t transmitText(char *data, uint64_t *id); // Send ASCII string. Assigned message ID is returned in id.
  SWARM_M138_error_t transmitText(char *data, uint64_t *id, uint16_t appID); // Send ASCII string. Assigned message ID is returned in id.
  SWARM_M138_error_t transmitTextHold(char *data, uint64_t *id, uint32_t hold); // Send ASCII string. Assigned message ID is returned in id. Hold for up to hold seconds
  SWARM_M138_error_t transmitTextHold(char *data, uint64_t *id, uint32_t hold, uint16_t appID); // Send ASCII string. Assigned message ID is returned in id. Hold for up to hold seconds
  SWARM_M138_error_t transmitTextExpire(char *data, uint64_t *id, uint32_t epoch); // Send ASCII string. Assigned message ID is returned in id. Expire message at epoch
  SWARM_M138_error_t transmitTextExpire(char *data, uint64_t *id, uint32_t epoch, uint16_t appID); // Send ASCII string. Assigned message ID is returned in id. Expire message at epoch
  SWARM_M138_error_t transmitBinary(char *data, uint8_t len, uint64_t *id); // Send binary data. Assigned message ID is returned in id.
  SWARM_M138_error_t transmitBinary(char *data, uint8_t len, uint64_t *id, uint16_t appID); // Send binary data. Assigned message ID is returned in id.
  SWARM_M138_error_t transmitBinaryHold(char *data, uint8_t len, uint64_t *id, uint32_t hold); // Send binary data. Assigned message ID is returned in id. Hold for up to hold seconds
  SWARM_M138_error_t transmitBinaryHold(char *data, uint8_t len, uint64_t *id, uint32_t hold, uint16_t appID); // Send binary data. Assigned message ID is returned in id. Hold for up to hold seconds
  SWARM_M138_error_t transmitBinaryExpire(char *data, uint8_t len, uint64_t *id, uint32_t epoch); // Send binary data. Assigned message ID is returned in id. Expire message at epoch
  SWARM_M138_error_t transmitBinaryExpire(char *data, uint8_t len, uint64_t *id, uint32_t epoch, uint16_t appID); // Send binary data. Assigned message ID is returned in id. Expire message at epoch

  /**  Process unsolicited messages from the modem */
  bool checkUnsolicitedMsg(void);

  /** Callbacks (called by checkUnsolicitedMsg) */
  void setDateTimeCallback(void (*socketListenCallback)(int, IPAddress, unsigned int, int, IPAddress, unsigned int)); // listen Socket, local IP Address, listen Port, socket, remote IP Address, port

  /** Direct write/print to modem serial port */
  virtual size_t write(uint8_t c);
  virtual size_t write(const char *str);
  virtual size_t write(const char *buffer, size_t size);

private:
  HardwareSerial *_hardSerial;
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  SoftwareSerial *_softSerial;
#endif

  unsigned long _baud;

  TwoWire* _i2cPort;
  byte _address;

  Stream *_debugPort;       //The stream to send debug messages to if enabled. Usually Serial.
  bool _printDebug = false; //Flag to print debugging variables

  int _gpio1Pin = SWARM_M138_GPIO1_PIN; // digital pin connected to GPIO1 (if provided)

  bool _checkUnsolicitedMsgReentrant = false; // Prevent reentry of checkUnsolicitedMsg - just in case it gets called from a callback

  #define _RxBuffSize 512
  const unsigned long _rxWindowMillis = 2; // 1ms is not quite long enough for a single char at 9600 baud. millis roll over much less often than micros.
  char *_swarmRxBuffer; // Allocated in SWARM_M138::begin
  char *_pruneBuffer;
  char *_swarmBacklog;

  // Callbacks for unsolicited messages
  void (*_swarmDateTimeCallback)(Swarm_M138_DateTimeData_t *dateTime);

  SWARM_M138_error_t init(unsigned long baud, SWARM_M138_init_type_t initType = SWARM_M138_INIT_STANDARD);

  // Wait for an expected response (don't send a command)
  SWARM_M138_error_t waitForResponse(const char *expectedResponse, const char *expectedError, uint16_t timeout);

  // Send command with an expected (potentially partial) response or error
  SWARM_M138_error_t sendCommandWithResponse(const char *command, const char *expectedResponse, const char *possibleError,
                                             unsigned long commandTimeout = SWARM_M138_STANDARD_RESPONSE_TIMEOUT);

  // Send a command
  void sendCommand(const char *command);

  SWARM_M138_error_t parseDateTimeIndication(char *dateTime);

  // UART Functions
  size_t hwPrint(const char *s);
  size_t hwWriteData(const char *buff, int len);
  size_t hwWrite(const char c);
  int readAvailable(char *inString);
  char readChar(void);
  int hwAvailable(void);
  void beginSerial(unsigned long baud);
  void setTimeout(unsigned long timeout);
  bool find(char *target);

  char *swarm_m138_calloc_char(size_t num);

  bool initializeBuffers();
  bool processUnsolicitedEvent(const char *event);
  void pruneBacklog(void);

  // GPS Helper functions
  char *readDataUntil(char *destination, unsigned int destSize, char *source, char delimiter);
  bool parseGeospatial(char *geospatialString, PositionData *pos, ClockData *clk, SpeedData *spd);
};

#endif //SPARKFUN_SWARM_M138_ARDUINO_LIBRARY_H
