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

#ifdef ARDUINO_ARCH_AVR                    // Arduino AVR boards (Uno, Pro Micro, etc.)
#define SWARM_M138_SOFTWARE_SERIAL_ENABLED // Enable software serial
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

#ifdef ARDUINO_ARCH_STM32                  // STM32 based boards (Disco, Nucleo, etc)
#define SWARM_M138_SOFTWARE_SERIAL_ENABLED // Enable software serial
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

#include <Wire.h> // Needed for I2C communication with Qwiic Swarm

/** Timeouts for the serial commands */
#define SWARM_M138_STANDARD_RESPONSE_TIMEOUT 1000 ///< Standard command timeout: allow one second for the modem to respond
#define SWARM_M138_MESSAGE_DELETE_TIMEOUT 5000    ///< Allow extra time when deleting a message
#define SWARM_M138_MESSAGE_ID_TIMEOUT 5000        ///< Allow extra time when reading the message IDs
#define SWARM_M138_MESSAGE_READ_TIMEOUT 3000      ///< Allow extra time when reading a message
#define SWARM_M138_MESSAGE_TRANSMIT_TIMEOUT 3000  ///< Allow extra time when queueing a message for transmission

/** Modem Serial Baud Rate */
#define SWARM_M138_SERIAL_BAUD_RATE 115200 ///< The modem serial baud rate is 115200 and cannot be changed

/** Default I2C address used by the Qwiic Swarm Breakout. Can be changed. */
#define SFE_QWIIC_SWARM_DEFAULT_I2C_ADDRESS 0x52 ///< The default I2C address for the SparkFun Qwiic Swarm Breakout

/** Swarm packet length */
#define SWARM_M138_MAX_PACKET_LENGTH_BYTES 192 ///< The maximum packet length - defined as binary bytes
#define SWARM_M138_MAX_PACKET_LENGTH_HEX 384   ///< The maximum packet length - encoded as ASCII Hex

/** Minimum memory allocations for each message type */
#define SWARM_M138_MEM_ALLOC_CS 30  ///< E.g. DI=0x001abe,DN=M138 . Should be 20 but maybe the modem model could be longer than 4 bytes?
#define SWARM_M138_MEM_ALLOC_FV 37  ///< E.g. 2021-12-14T21:27:41,v1.5.0-rc4 . Should be 31 but maybe each v# could be three digits?
#define SWARM_M138_MEM_ALLOC_MS 128 ///< Allocate enough storage to hold the $M138 Modem Status debug or error text. GUESS! TO DO: confirm the true max length

/** Suported Commands */
const char SWARM_M138_COMMAND_CONFIGURATION[] = "$CS";   ///< Configuration Settings
const char SWARM_M138_COMMAND_DATE_TIME_STAT[] = "$DT";  ///< Date/Time Status
const char SWARM_M138_COMMAND_FIRMWARE_VER[] = "$FV";    ///< Retrieve Firmware Version
const char SWARM_M138_COMMAND_GPS_JAMMING[] = "$GJ";     ///< GPS Jamming/Spoofing Indication
const char SWARM_M138_COMMAND_GEOSPATIAL_INFO[] = "$GN"; ///< Geospatial Information
const char SWARM_M138_COMMAND_GPIO1_CONTROL[] = "$GP";   ///< GPIO1 Control
const char SWARM_M138_COMMAND_GPS_FIX_QUAL[] = "$GS";    ///< GPS Fix Quality
const char SWARM_M138_COMMAND_MSG_RX_MGMT[] = "$MM";     ///< Messages Received Management
const char SWARM_M138_COMMAND_MSG_TX_MGMT[] = "$MT";     ///< Messages to Transmit Management
const char SWARM_M138_COMMAND_POWER_OFF[] = "$PO";       ///< Power Off
const char SWARM_M138_COMMAND_POWER_STAT[] = "$PW";      ///< Power Status
const char SWARM_M138_COMMAND_RX_DATA_MSG[] = "$RD";     ///< Receive Data Message
const char SWARM_M138_COMMAND_RESTART[] = "$RS";         ///< Restart Device
const char SWARM_M138_COMMAND_RX_TEST[] = "$RT";         ///< Receive Test
const char SWARM_M138_COMMAND_SLEEP[] = "$SL";           ///< Sleep Mode
const char SWARM_M138_COMMAND_MODEM_STAT[] = "$M138";    ///< Modem Status
const char SWARM_M138_COMMAND_TX_DATA[] = "$TD";         ///< Transmit Data

/** An enum defining the command result */
typedef enum
{
  SWARM_M138_ERROR_ERROR = 0,        ///< Just a plain old communication error
  SWARM_M138_ERROR_SUCCESS,          ///< Hey, it worked!
  SWARM_M138_ERROR_MEM_ALLOC,        ///< Memory allocation error
  SWARM_M138_ERROR_TIMEOUT,          ///< Communication timeout
  SWARM_M138_ERROR_INVALID_FORMAT,   ///< Indicates the command response format was invalid
  SWARM_M138_ERROR_INVALID_CHECKSUM, ///< Indicates the command response checksum was invalid
  SWARM_M138_ERROR_INVALID_RATE,     ///< Indicates the message rate was invalid
  SWARM_M138_ERROR_INVALID_MODE,     ///< Indicates the GPIO1 pin mode was invalid
  SWARM_M138_ERROR_ERR               ///< Command input error (ERR) - the error is copied into commandError
} Swarm_M138_Error_e;
#define SWARM_M138_SUCCESS SWARM_M138_ERROR_SUCCESS ///< Hey, it worked!

/** Define the maximum message 'rate' (interval) */
const uint32_t SWARM_M138_MAX_MESSAGE_RATE = 0x7FFFFFFF; ///< 2147483647 (2^31 - 1)

/** A struct to hold the date and time returned by $DT */
typedef struct
{
  uint16_t YYYY; // Year:    1970..2099
  uint8_t MM;    // Month:   01..12
  uint8_t DD;    // Day:     01..31
  uint8_t hh;    // Hour:    00..23
  uint8_t mm;    // Minutes: 00..59
  uint8_t ss;    // Seconds: 00..59
  bool valid;    // flag: I == Invalid == false; V == Valid == true
} Swarm_M138_DateTimeData_t;

/** A struct to hold the GPS jamming / spoofing indication */
typedef struct
{
  uint8_t spoof_state;   // 0 Spoofing unknown or deactivated
                         // 1 No spoofing indicated
                         // 2 Spoofing indicated
                         // 3 Multiple spoofing indications
  uint8_t jamming_level; // 0 = no CW jamming, 255 = strong CW jamming
} Swarm_M138_GPS_Jamming_Indication_t;

/** A struct to hold the geospatial data returned by $GN */
typedef struct
{
  float lat;    // Degrees: +/- 90
  float lon;    // Degrees: +/- 180
  float alt;    // m
  float course; // Degrees: 0..359 : 0=north, 90=east, 180=south, and 270=west
  float speed;  // km/h
} Swarm_M138_GeospatialData_t;

/** Enum for the GPIO1 pin modes */
typedef enum
{
  SWARM_M138_GPIO1_ANALOG = 0,
  SWARM_M138_GPIO1_EXIT_SLEEP_LOW_HIGH,
  SWARM_M138_GPIO1_EXIT_SLEEP_HIGH_LOW,
  SWARM_M138_GPIO1_OUTPUT_LOW,
  SWARM_M138_GPIO1_OUTPUT_HIGH,
  SWARM_M138_GPIO1_MESSAGES_PENDING_LOW,
  SWARM_M138_GPIO1_MESSAGES_PENDING_HIGH,
  SWARM_M138_GPIO1_SLEEP_MODE_LOW,
  SWARM_M138_GPIO1_SLEEP_MODE_HIGH,
  SWARM_M138_GPIO1_INVALID
} Swarm_M138_GPIO1_Mode_e;

/** Enum for the GPS fix type */
typedef enum
{
  SWARM_M138_GPS_FIX_TYPE_NF = 0, // No Fix
  SWARM_M138_GPS_FIX_TYPE_DR,     // Dead reckoning only solution
  SWARM_M138_GPS_FIX_TYPE_G2,     // Standalone 2D solution
  SWARM_M138_GPS_FIX_TYPE_G3,     // Standalone 3D solution
  SWARM_M138_GPS_FIX_TYPE_D2,     // Differential 2D solution
  SWARM_M138_GPS_FIX_TYPE_D3,     // Differential 3D solution
  SWARM_M138_GPS_FIX_TYPE_RK,     // Combined GNSS + dead reckoning solution
  SWARM_M138_GPS_FIX_TYPE_TT,     // Time only solution
  SWARM_M138_GPS_FIX_TYPE_INVALID
} Swarm_M138_GPS_Fix_Type_e;

/** A struct to hold the GPS fix quality */
typedef struct
{
  uint16_t hdop;     // Horizontal dilution of precision (0..9999) (integer = actual hdop * 100)
  uint16_t vdop;     //  Vertical dilution of precision (0..9999) (integer = actual vdop * 100)
  uint8_t gnss_sats; // Number of GNSS satellites used in solution (integer)
  uint8_t unused;    // Always reads as 0, unused
  Swarm_M138_GPS_Fix_Type_e fix_type;
} Swarm_M138_GPS_Fix_Quality_t;

/** A struct to hold the power staus info */
typedef struct
{
  float unused1;
  float unused2;
  float unused3;
  float unused4;
  float temp; // CPU Temperature in degrees C to one decimal point
} Swarm_M138_Power_Status_t;

/** A struct to hold the receive test results */
typedef struct
{
  bool background;                // If true: the struct holds the rssi_background only. If false: the struct holds everything except rssi_background.
  int16_t rssi_background;        // Received background noise signal strength in dBm for open channel (integer).
  int16_t rssi_sat;               // Received signal strength in dBm for packet from satellite (integer)
  int16_t snr;                    // Signal to noise ratio in dB for packet (integer)
  int16_t fdev;                   // Frequency deviation in Hz for packet (integer)
  Swarm_M138_DateTimeData_t time; // Date and time (UTC) of received packet (valid flag is unused - always set to true)
  uint32_t sat_id;                // Device ID of satellite heard (hexadecimal)
} Swarm_M138_Receive_Test_t;

/** An enum for the sleep mode wake cause */
typedef enum
{
  SWARM_M138_WAKE_CAUSE_GPIO = 0, // GPIO input changed from inactive to active state
  SWARM_M138_WAKE_CAUSE_SERIAL,   // Activity was detected on the RX pin of the Modem's UART
  SWARM_M138_WAKE_CAUSE_TIME,     // The S or U parameter time has been reached
  SWARM_M138_WAKE_CAUSE_INVALID
} Swarm_M138_Wake_Cause_e;

/** An enum for the modem status */
typedef enum
{
  SWARM_M138_MODEM_STATUS_BOOT_ABORT = 0, // A firmware crash occurred that caused a restart
  SWARM_M138_MODEM_STATUS_BOOT_POWERON,   // Power has been applied
  SWARM_M138_MODEM_STATUS_BOOT_RUNNING,   // Boot has completed and ready to accept commands
  SWARM_M138_MODEM_STATUS_BOOT_UPDATED,   // A firmware update was performed
  SWARM_M138_MODEM_STATUS_BOOT_VERSION,   // Current firmware version information
  SWARM_M138_MODEM_STATUS_BOOT_RESTART,   // Modem is restarting after $RS Restart Device
  SWARM_M138_MODEM_STATUS_BOOT_SHUTDOWN,  // Modem has shutdown after $PO Power Off. Disconnect and reconnect power to restart
  SWARM_M138_MODEM_STATUS_DATETIME,       // The first time GPS has acquired a valid date/time reference
  SWARM_M138_MODEM_STATUS_POSITION,       // The first time GPS has acquired a valid position 3D fix
  SWARM_M138_MODEM_STATUS_DEBUG,          // Debug message (data - debug text)
  SWARM_M138_MODEM_STATUS_ERROR,          // Error message (data - error text)
  SWARM_M138_MODEM_STATUS_UNKNOWN,        // A new, undocumented message
  SWARM_M138_MODEM_STATUS_INVALID
} Swarm_M138_Modem_Status_e;

/** Communication interface for the Swarm M138 satellite modem. */
class SWARM_M138
{
public:
  // Constructor
  /** @brief Class to communicate with the Swarm M138 satellite modem */
  SWARM_M138(void);

  /** Begin -- initialize module and ensure it's connected */
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  bool begin(SoftwareSerial &softSerial);
#endif
  bool begin(HardwareSerial &hardSerial);
  bool begin(byte deviceAddress = SFE_QWIIC_SWARM_DEFAULT_I2C_ADDRESS, TwoWire &wirePort = Wire);

  /** Debug prints */
  void enableDebugging(Stream &debugPort = Serial); // Turn on debug printing. If user doesn't specify then Serial will be used.
  void disableDebugging(void);                      // Turn off debug printing

  /** Commands */

  /** Configuration Settings */
  Swarm_M138_Error_e getConfigurationSettings(char *settings); // Get the Swarm device ID and type name
  Swarm_M138_Error_e getDeviceID(uint32_t *id);                // Get the Swarm device ID
  bool isConnected(void);                                      // isConnected calls getDeviceID

  /** Date/Time */
  Swarm_M138_Error_e getDateTime(Swarm_M138_DateTimeData_t *dateTime); // Get the most recent $DT message
  Swarm_M138_Error_e getDateTimeRate(uint32_t *rate);                  // Query the current $DT rate
  Swarm_M138_Error_e setDateTimeRate(uint32_t rate);                   // Set the rate of $DT messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Firmware Version */
  Swarm_M138_Error_e getFirmwareVersion(char *version); // Get the Swarm device firmware version

  /** GPS Jamming/Spoofing Indication */
  Swarm_M138_Error_e getGpsJammingIndication(Swarm_M138_GPS_Jamming_Indication_t *jamming); // Get the most recent $GJ message
  Swarm_M138_Error_e getGpsJammingIndicationRate(uint32_t *rate);                           // Query the current $GJ rate
  Swarm_M138_Error_e setGpsJammingIndicationRate(uint32_t rate);                            // Set the rate of $GJ messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Geospatial information */
  Swarm_M138_Error_e getGeospatialInfo(Swarm_M138_GeospatialData_t *info); // Get the most recent $GN message
  Swarm_M138_Error_e getGeospatialInfoRate(uint32_t *rate);                // Query the current $GN rate
  Swarm_M138_Error_e setGeospatialInfoRate(uint32_t rate);                 // Set the rate of $GN messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** GPIO1 Control */
  Swarm_M138_Error_e getGPIO1Mode(Swarm_M138_GPIO1_Mode_e *mode); // Get the GPIO1 pin mode
  Swarm_M138_Error_e setGPIO1Mode(Swarm_M138_GPIO1_Mode_e mode);  // Set the GPIO1 pin mode

  /** GPS fix quality */
  Swarm_M138_Error_e getGpsFixQuality(Swarm_M138_GPS_Fix_Quality_t *fixQuality); // Get the most recent $GS message
  Swarm_M138_Error_e getGpsFixQualityRate(uint32_t *rate);                       // Query the current $GS rate
  Swarm_M138_Error_e setGpsFixQualityRate(uint32_t rate);                        // Set the rate of $GS messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Power Off */
  Swarm_M138_Error_e powerOff(void); // The Modem enters a low power mode until power is completely removed and restored

  /** Power Status */
  Swarm_M138_Error_e getPowerStatus(Swarm_M138_Power_Status_t *powerStatus); // Get the most recent $PW message
  Swarm_M138_Error_e getPowerStatusRate(uint32_t *rate);                     // Query the current $PW rate
  Swarm_M138_Error_e setPowerStatusRate(uint32_t rate);                      // Set the rate of $PW messages. 0 == Disable. Max is 2147483647 (2^31 - 1)
  Swarm_M138_Error_e getTemperature(float *temperature);                     // Get the most recent temperature

  /** Restart Device */
  Swarm_M138_Error_e restartDevice(bool dbinit = false); // Restart the modem. Optionally clear the message database, to clear the DBXTOHIVEFULL error

  /** Receive Test */
  Swarm_M138_Error_e getReceiveTest(Swarm_M138_Receive_Test_t *rxTest); // Get the most recent $RT message
  Swarm_M138_Error_e getReceiveTestRate(uint32_t *rate);                // Query the current $RT rate
  Swarm_M138_Error_e setReceiveTestRate(uint32_t rate);                 // Set the rate of $RT messages. 0 == Disable. Max is 2147483647 (2^31 - 1)

  /** Sleep Mode */
  Swarm_M138_Error_e sleepMode(uint32_t seconds);                                              // Sleep for this many seconds
  Swarm_M138_Error_e sleepMode(Swarm_M138_DateTimeData_t sleepUntil, bool dateAndTime = true); // Sleep until this date and time. Set dateAndTime to false to sleep until a time

  /** Messages Received Management */
  Swarm_M138_Error_e getRxMessageCount(uint16_t *count, bool unread = false);                                             // Return count of all messages (default) or unread messages (unread = true)
  Swarm_M138_Error_e deleteRxMessage(uint64_t msg_id);                                                                    // Delete RX message with ID
  Swarm_M138_Error_e deleteAllRxMessages(bool read = true);                                                               // Delete all read RX messages (default) or all messages (read = false)
  Swarm_M138_Error_e markRxMessage(uint64_t msg_id);                                                                      // Mark message ID as read
  Swarm_M138_Error_e markAllRxMessages(void);                                                                             // Mark all messages as read
  Swarm_M138_Error_e getMessageNotifications(bool *enabled);                                                              // Query if message notifications are enabled
  Swarm_M138_Error_e setMessageNotifications(bool enable);                                                                // Enable / disable message notifications
  Swarm_M138_Error_e readMessage(uint64_t msg_id, char *asciiHex, size_t len, uint32_t *epoch = NULL, uint16_t *appID = NULL);        // Read the message with ID. Message contents are copied to asciiHex as ASCII Hex
  Swarm_M138_Error_e readOldestMessage(char *asciiHex, size_t len, uint64_t *msg_id, uint32_t *epoch = NULL, uint16_t *appID = NULL); // Read the oldest message. Message contents are copied to asciiHex. ID is copied to id.
  Swarm_M138_Error_e readNewestMessage(char *asciiHex, size_t len, uint64_t *msg_id, uint32_t *epoch = NULL, uint16_t *appID = NULL); // Read the oldest message. Message contents are copied to asciiHex. ID is copied to id.

  /** Messages To Transmit Management */
  Swarm_M138_Error_e getUnsentMessageCount(uint16_t *count);                                                                     // Return count of all unsent messages
  Swarm_M138_Error_e deleteTxMessage(uint64_t msg_id);                                                                           // Delete TX message with ID
  Swarm_M138_Error_e deleteAllTxMessages(void);                                                                                  // Delete all unsent messages
  Swarm_M138_Error_e listTxMessage(uint64_t msg_id, char *asciiHex, size_t len, uint32_t *epoch = NULL, uint16_t *appID = NULL); // List unsent message with ID
  Swarm_M138_Error_e listTxMessagesIDs(uint64_t *ids, uint16_t maxCount);                                                        // List the IDs of all unsent messages. Call getUnsentMessageCount first so you know how many IDs to expect

  /** Transmit Data */
  // The application ID is optional. Valid appID's are: 0 to 64999. Swarm reserves use of 65000 - 65535.
  Swarm_M138_Error_e transmitText(const char *data, uint64_t *msg_id);                                                        // Send ASCII string. Assigned message ID is returned in id.
  Swarm_M138_Error_e transmitText(const char *data, uint64_t *msg_id, uint16_t appID);                                        // Send ASCII string. Assigned message ID is returned in id.
  Swarm_M138_Error_e transmitTextHold(const char *data, uint64_t *msg_id, uint32_t hold);                                     // Send ASCII string. Assigned message ID is returned in id. Hold for up to hold seconds
  Swarm_M138_Error_e transmitTextHold(const char *data, uint64_t *msg_id, uint32_t hold, uint16_t appID);                     // Send ASCII string. Assigned message ID is returned in id. Hold for up to hold seconds
  Swarm_M138_Error_e transmitTextExpire(const char *data, uint64_t *msg_id, uint32_t epoch);                                  // Send ASCII string. Assigned message ID is returned in id. Expire message at epoch
  Swarm_M138_Error_e transmitTextExpire(const char *data, uint64_t *msg_id, uint32_t epoch, uint16_t appID);                  // Send ASCII string. Assigned message ID is returned in id. Expire message at epoch
  Swarm_M138_Error_e transmitBinary(const uint8_t *data, size_t len, uint64_t *msg_id);                                       // Send binary data. Assigned message ID is returned in id.
  Swarm_M138_Error_e transmitBinary(const uint8_t *data, size_t len, uint64_t *msg_id, uint16_t appID);                       // Send binary data. Assigned message ID is returned in id.
  Swarm_M138_Error_e transmitBinaryHold(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t hold);                    // Send binary data. Assigned message ID is returned in id. Hold for up to hold seconds
  Swarm_M138_Error_e transmitBinaryHold(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t hold, uint16_t appID);    // Send binary data. Assigned message ID is returned in id. Hold for up to hold seconds
  Swarm_M138_Error_e transmitBinaryExpire(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t epoch);                 // Send binary data. Assigned message ID is returned in id. Expire message at epoch
  Swarm_M138_Error_e transmitBinaryExpire(const uint8_t *data, size_t len, uint64_t *msg_id, uint32_t epoch, uint16_t appID); // Send binary data. Assigned message ID is returned in id. Expire message at epoch

  /**  Process unsolicited messages from the modem. Call the callbacks if required */
  bool checkUnsolicitedMsg(void);

  /** Callbacks (called by checkUnsolicitedMsg) */
  void setDateTimeCallback(void (*swarmDateTimeCallback)(const Swarm_M138_DateTimeData_t *dateTime));                                                                             // Set callback for $DT
  void setGpsJammingCallback(void (*swarmGpsJammingCallback)(const Swarm_M138_GPS_Jamming_Indication_t *jamming));                                                                // Set callback for $GJ
  void setGeospatialInfoCallback(void (*swarmGeospatialCallback)(const Swarm_M138_GeospatialData_t *info));                                                                       // Set callback for $GN
  void setGpsFixQualityCallback(void (*swarmGpsFixQualityCallback)(const Swarm_M138_GPS_Fix_Quality_t *fixQuality));                                                              // Set callback for $GS
  void setPowerStatusCallback(void (*swarmPowerStatusCallback)(const Swarm_M138_Power_Status_t *status));                                                                         // Set callback for $PW
  void setReceiveMessageCallback(void (*swarmReceiveMessageCallback)(const uint16_t *appID, const int16_t *rssi, const int16_t *snr, const int16_t *fdev, const char *asciiHex)); // Set callback for $RD
  void setReceiveTestCallback(void (*swarmReceiveTestCallback)(const Swarm_M138_Receive_Test_t *rxTest));                                                                         // Set callback for $RT
  void setSleepWakeCallback(void (*swarmSleepWakeCallback)(Swarm_M138_Wake_Cause_e cause));                                                                                       // Set callback for $SL WAKE
  void setModemStatusCallback(void (*swarmModemStatusCallback)(Swarm_M138_Modem_Status_e status, const char *data));                                                              // Set callback for $M138. data could be NULL for messages like BOOT_RUNNING
  void setTransmitDataCallback(void (*swarmTransmitDataCallback)(const int16_t *rssi_sat, const int16_t *snr, const int16_t *fdev, const uint64_t *msg_id));                      // Set callback for $TD SENT

  /** Convert modem status enum etc. into printable text */
  const char *modemStatusString(Swarm_M138_Modem_Status_e status);
  const char *modemErrorString(Swarm_M138_Error_e error);
  const char *commandErrorString(const char *ERR);

/** Storage for the most recent command error */
#define SWARM_M138_MAX_CMD_ERROR_LEN 32 ///< Allocate 32 bytes to store the most recent command error
  char *commandError;

private:
  HardwareSerial *_hardSerial;
#ifdef SWARM_M138_SOFTWARE_SERIAL_ENABLED
  SoftwareSerial *_softSerial;
#endif

  unsigned long _baud; // Baud rate for serial communication with the modem

  Stream *_debugPort; // The stream to send debug messages to if enabled. Usually Serial.
  bool _printDebug;   // Flag to print debugging variables

  bool _checkUnsolicitedMsgReentrant; // Prevent reentry of checkUnsolicitedMsg - just in case it gets called from a callback

#define _RxBuffSize 512
  const unsigned long _rxWindowMillis = 5; // Wait up to 5ms for any more serial characters to arrive
  char *_swarmBacklog;                     // Allocated in SWARM_M138::begin

  // Callbacks for unsolicited messages
  void (*_swarmDateTimeCallback)(const Swarm_M138_DateTimeData_t *dateTime);
  void (*_swarmGpsJammingCallback)(const Swarm_M138_GPS_Jamming_Indication_t *jamming);
  void (*_swarmGeospatialCallback)(const Swarm_M138_GeospatialData_t *info);
  void (*_swarmGpsFixQualityCallback)(const Swarm_M138_GPS_Fix_Quality_t *fixQuality);
  void (*_swarmPowerStatusCallback)(const Swarm_M138_Power_Status_t *status);
  void (*_swarmReceiveMessageCallback)(const uint16_t *appID, const int16_t *rssi, const int16_t *snr, const int16_t *fdev, const char *asciiHex);
  void (*_swarmReceiveTestCallback)(const Swarm_M138_Receive_Test_t *rxTest);
  void (*_swarmSleepWakeCallback)(Swarm_M138_Wake_Cause_e cause);
  void (*_swarmModemStatusCallback)(Swarm_M138_Modem_Status_e status, const char *data);
  void (*_swarmTransmitDataCallback)(const int16_t *rssi_sat, const int16_t *snr, const int16_t *fdev, const uint64_t *id);

  // Add the two NMEA checksum bytes and line feed to a command
  void addChecksumLF(char *command);

  // Check if the response / message format and checksum is valid
  Swarm_M138_Error_e checkChecksum(char *startPosition);

  // Extract the error from the command response
  Swarm_M138_Error_e extractCommandError(char *startPosition);

  // Send command with the start of an expected response
  Swarm_M138_Error_e sendCommandWithResponse(const char *command, const char *expectedResponseStart, const char *expectedErrorStart,
                                             char *responseDest, size_t destSize, unsigned long commandTimeout = SWARM_M138_STANDARD_RESPONSE_TIMEOUT);

  // Send a command (don't wait for a response)
  void sendCommand(const char *command);

  // Wait for an expected response or error (don't send a command)
  Swarm_M138_Error_e waitForResponse(const char *expectedResponseStart, const char *expectedErrorStart,
                                     char *responseDest, size_t destSize, unsigned long timeout = SWARM_M138_STANDARD_RESPONSE_TIMEOUT);

  // Queue a text message for transmission
  Swarm_M138_Error_e transmitText(const char *data, uint64_t *msg_id, bool useAppID, uint16_t appID,
                                  bool useHold, uint32_t hold, bool useEpoch, uint32_t epoch);

  // Queue a binary message for transmission
  Swarm_M138_Error_e transmitBinary(const uint8_t *data, size_t len, uint64_t *msg_id, bool useAppID, uint16_t appID,
                                    bool useHold, uint32_t hold, bool useEpoch, uint32_t epoch);

  // Common code for readMessage / readOldestMessage / readNewestMessage
  Swarm_M138_Error_e readMessageInternal(const char mode, uint64_t msg_id_in, char *asciiHex, size_t len, uint64_t *msg_id_out, uint32_t *epoch, uint16_t *appID);

  bool initializeBuffers(void);
  bool processUnsolicitedEvent(const char *event);
  void pruneBacklog(void);

  // Support for Qwiic Swarm

  TwoWire *_i2cPort;                                   // The I2C (Wire) port for the Qwiic Swarm
  byte _address;                                       // I2C address of the Qwiic Swarm
  int qwiicSwarmAvailable(void);                       // Check how many serial bytes Qwiic Sawrm has in its buffer
  int qwiicSwarmReadChars(int len, char *dest);        // Read bytes from Qwiic Swarm
  int qwiicSwarmWriteChars(int len, const char *dest); // Write bytes to Qwiic Swarm
  unsigned long _lastI2cCheck;
#define QWIIC_SWARM_I2C_POLLING_WAIT_MS 2 // Avoid pounding the I2C bus. Wait at least 2ms between calls to qwiicSwarmAvailable
// Define the I2C 'registers'
#define QWIIC_SWARM_LEN_REG 0xFD  // The serial length regsiter: 2 bytes (MSB, LSB) indicating how many serial characters are available to be read
#define QWIIC_SWARM_DATA_REG 0xFF // The serial data register: used to read and write serial data from/to the modem
// Define the maximum number of serial bytes to be requested from the ATtiny841
#define QWIIC_SWARM_SER_PACKET_SIZE 8
// Qwiic Iridium ATtiny841 I2C buffer length
#define QWIIC_SWARM_I2C_BUFFER_LENGTH 32

  // Memory allocation

  char *swarm_m138_alloc_char(size_t num);
  void swarm_m138_free_char(char *freeMe);

  // UART / I2C Functions
  size_t hwPrint(const char *s);
  size_t hwWriteData(const char *buff, int len);
  size_t hwWrite(const char c);
  int hwAvailable(void);
  int hwReadChars(char *buf, int len);

  void beginSerial(unsigned long baud);
};

#endif // SPARKFUN_SWARM_M138_ARDUINO_LIBRARY_H
