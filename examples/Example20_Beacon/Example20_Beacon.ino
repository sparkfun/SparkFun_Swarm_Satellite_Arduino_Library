/*!
 * @file Example20_Beacon.ino
 * 
 * @mainpage SparkFun Swarm Satellite Arduino Library
 * 
 * @section intro_sec Examples
 * 
 * This example shows how to:
 *   Create a beacon using the Swarm M138 modem
 *   The modem's location and other information is added to the TX queue every txIntervalMins minutes
 *   The loop waits for the TX queue to be empty and then puts the modem to sleep until the next transmit
 *   The code also demonstrates how to configure to modem's GPIO1 pin so it could be used to wake the processor
 * 
 * Want to support open source hardware? Buy a board from SparkFun!
 * SparkX Swarm Serial Breakout : https://www.sparkfun.com/products/19236
 * 
 * @section author Author
 * 
 * This library was written by:
 * Paul Clark
 * SparkFun Electronics
 * June 2022
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

#define console Serial // Use Serial for the console / user interface. Change this if required.

const uint16_t txIntervalMins = 60; // Queue a new message every hour

// Define which digital I//O pin is connected to the modem's GPIO1 pin
// The code will set up an interrupt on this pin to show when the modem comes out of sleep
// Change this if required
// Set to -1 if you do not want to use GPIO1
const int gpio1Pin = 21;

Swarm_M138_Modem_Status_e modemStatus = SWARM_M138_MODEM_STATUS_INVALID; // The modem's status from the most recent $M138 message

// Flag to indicate that "Boot has completed and ready to accept commands".
// "The customer application should wait until the boot process is complete and it has received the
//  $M138 BOOT,RUNNING*2a message before executing any commands."
bool runningSeen = false;

// Flag to indicate that "The first time GPS has acquired a valid date/time reference".
// "In order to send a $TD command, you must first wait for a $M138 DATETIME*56 response after power up."
bool datetimeSeen = false;

// Flag to indicate that "the GPS has obtained a valid position reference as indicated by the $M138 POSITION*4e message"
bool positionSeen = false;

// Flag to indicate that GPIO1 has generated an interrupt (i.e. the modem has woken from sleep)
volatile bool gpio1InterruptSeen = false;

// Flag to indicate that a $SL WAKE,TIME message has been received (i.e. the modem has woken from sleep)
bool sleepWakeSeen = false;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// The loop is a 'state machine' defined in a switch statement. Here are the states:

enum loopStates
{
  startUp = 0,
  waitForRunningSeen,
  configureModem,
  waitForDateTimePosition,
  waitFor3Dposition,
  queueMessage,
  waitForTx,
  goToSleep,
  sleepUntil
};

loopStates loopState = startUp; // loopState controls the state machine state

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: processM138Msg will be called when a new unsolicited $M138 message arrives.
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_Modem_Status_e.

void processM138Msg(Swarm_M138_Modem_Status_e status, const char *data)
{
  console.print(F("New $M138 message received: "));
  console.print((int)status);
  console.print(F(" : "));
  console.print(mySwarm.modemStatusString(status));
  if (*data != 0) // data could be NULL for messages like BOOT_RUNNING
  {
    console.print(F(" : "));
    console.print(data);
  }
  console.println();

  modemStatus = status; // Record the modem status
  
  if (status == SWARM_M138_MODEM_STATUS_BOOT_RUNNING)
    runningSeen = true;

  if (status == SWARM_M138_MODEM_STATUS_DATETIME)
    datetimeSeen = true;

  if (status == SWARM_M138_MODEM_STATUS_POSITION)
    positionSeen = true;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: processSleepWakeMsg will be called when a new unsolicited $SL message arrives.
// See SparkFun_Swarm_Satellite_Arduino_Library.h for the full definition of Swarm_M138_Modem_Status_e.

void processSleepWakeMsg(Swarm_M138_Wake_Cause_e cause)
{
  console.print(F("New $SL WAKE message received. Cause is: "));
  console.print((int)cause);
  console.print(F(" : "));
  if (cause == SWARM_M138_WAKE_CAUSE_GPIO)
    console.println(F("GPIO input changed from inactive to active state"));
  else if (cause == SWARM_M138_WAKE_CAUSE_SERIAL)
    console.println(F("Activity was detected on the RX pin of the Modem's UART"));
  else if (cause == SWARM_M138_WAKE_CAUSE_TIME)
    console.println(F("The S or U parameter time has been reached"));
  else // if (cause == SWARM_M138_WAKE_CAUSE_INVALID)
    console.println(F("UNKNOWN / INVALID"));

  sleepWakeSeen = true;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// GPIO1 Interrupt Service Routine

void gpio1ISR()
{
  gpio1InterruptSeen = true;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(2000); // Power up delay
  
  console.begin(115200);
  console.println(F("Swarm Satellite Beacon"));
  console.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  Swarm_M138_DateTimeData_t dateTime;
  Swarm_M138_GeospatialData_t pos;
  Swarm_M138_GPS_Fix_Quality_t fix;
  uint32_t secsToSleep;

  switch (loopState) // loop is one big switch statement
  {

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // startUp
    //
    // Begin communication with the modem
    // Set up the $M138 modem status callback
    // Restart the modem so we can capture the modem status messages as it boots
    
    case startUp:
    {
      //mySwarm.enableDebugging(console); // Uncomment this line to enable debug messages on console

      bool modemBegun = mySwarm.begin(swarmSerial); // Begin communication with the modem
      
      while (!modemBegun) // If the begin failed, keep trying to begin communication with the modem
      {
        console.println(F("Could not communicate with the modem. Please check the serial connections..."));
        delay(2000);
        modemBegun = mySwarm.begin(swarmSerial);
      }

      // Clear the modem status
      modemStatus = SWARM_M138_MODEM_STATUS_INVALID;
      runningSeen = false;
      datetimeSeen = false;
      positionSeen = false;

      mySwarm.checkUnsolicitedMsg(); // Clear any messages from the backlog

      // Set up the callback for the modem status message. Call processM138Msg when a new $M138 message arrives
      console.println(F("Setting up the $M138 callback"));
      mySwarm.setModemStatusCallback(&processM138Msg);

      // Set up the callback for the sleep mode messages. Call processSleepWakeMsg when a new $SL message arrives
      console.println(F("Setting up the $SL callback"));
      mySwarm.setSleepWakeCallback(&processSleepWakeMsg);

      // Restart the modem. Follow the modem status messages as it boots
      console.println(F("Restarting the modem"));
      Swarm_M138_Error_e err = mySwarm.restartDevice();

      // Check that the restart was acknowledged
      if (err == SWARM_M138_SUCCESS)
      {
        console.println(F("Modem is restarting..."));
        loopState = waitForRunningSeen; // Move on
      }
      else
      {
        console.println(F("Modem restart failed. Trying again..."));
        delay(2000);
        loopState = startUp; // Repeat startUp
      }
    }
    break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // waitForRunningSeen
    //
    // Wait until the $M138 BOOT,RUNNING message is received. Then we know the modem is ready to accept commands
    
    case waitForRunningSeen:
    {
      unsigned long waitStarted = millis();
      const unsigned long maxWait = 30000; // Timeout after 30 seconds

      while ((!runningSeen) && (millis() < (waitStarted + maxWait)))
      {
        mySwarm.checkUnsolicitedMsg(); // Process the $M138 messages when they arrive
      }

      if (runningSeen)
      {
        console.println(F("Modem is ready to accept commands"));
        loopState = configureModem; // Move on
      }
      else
      {
        console.println(F("BOOT,RUNNING not received. Trying again..."));
        delay(2000);
        loopState = startUp; // Repeat startUp
      }
    }
    break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // configureModem
    //
    // Disable any unsolicited messages
    // Configure the GPIO1 pin
    // Set up the wake-from-sleep interrupt
    
    case configureModem:
    {
      console.println(F("Disabling unsolicited messages"));

      Swarm_M138_Error_e err = mySwarm.setDateTimeRate(0);
      if (err == SWARM_M138_SUCCESS) err = mySwarm.setGpsJammingIndicationRate(0);
      if (err == SWARM_M138_SUCCESS) err = mySwarm.setGeospatialInfoRate(0);
      if (err == SWARM_M138_SUCCESS) err = mySwarm.setGpsFixQualityRate(0);
      if (err == SWARM_M138_SUCCESS) err = mySwarm.setPowerStatusRate(0);
      if (err == SWARM_M138_SUCCESS) err = mySwarm.setReceiveTestRate(0);
      if (err == SWARM_M138_SUCCESS) err = mySwarm.setMessageNotifications(false);

      if (err == SWARM_M138_SUCCESS)
      {
        // Configure GPIO1 so it is high during sleep and low when the modem is awake
        // This will create a falling edge which we can use as an interrupt if desired
        // E.g. the interrupt could be used to bring this processor out of sleep
        console.println(F("Configuring the GPIO1 pin"));
        err = mySwarm.setGPIO1Mode(SWARM_M138_GPIO1_SLEEP_MODE_HIGH);

        if (gpio1Pin >= 0) // Skip this if gpio1Pin is negative
        {
          pinMode(gpio1Pin, INPUT_PULLUP); // GPIO1 is an open-drain output. It needs a pull-up. (The SparkFun Breakout has a pull-up resistor)
          
          attachInterrupt(digitalPinToInterrupt(gpio1Pin), gpio1ISR, FALLING); // Set up the interrupt

          //attachInterrupt(gpio1Pin, gpio1ISR, FALLING); // Or use this if your platfrom does not support or need digitalPinToInterrupt()

          delay(1);
          gpio1InterruptSeen = false; // Clear the flag
        }
      }

      if (err == SWARM_M138_SUCCESS)
      {
        console.println(F("Modem is configured"));
        loopState = waitForDateTimePosition; // Move on
      }
      else
      {
        console.println(F("Could not configure the modem. Trying again..."));
        delay(2000);
        loopState = startUp; // Repeat startUp
      }
    }
    break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // waitForDateTimePosition
    //
    // Wait until the modem has both valid date-time and position
    
    case waitForDateTimePosition:
    {
      unsigned long waitStarted = millis();
      const unsigned long maxWait = 120000; // Timeout after 120 seconds

      console.println(F("Waiting for valid dateTime and position..."));

      while (((!datetimeSeen) || (!positionSeen)) && (millis() < (waitStarted + maxWait)))
      {
        mySwarm.checkUnsolicitedMsg(); // Process the $M138 messages when they arrive
      }

      if ((datetimeSeen) && (positionSeen))
      {
        console.println(F("Modem has valid dateTime and position"));
        loopState = waitFor3Dposition; // Move on
      }
      else
      {
        console.println(F("Modem does not have valid dateTime and position. Trying again..."));
        delay(2000);
        loopState = startUp; // Repeat startUp
      }      
    }
    break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // waitFor3Dposition
    //
    // Wait until the modem has both valid date-time and a 3D position
    
    case waitFor3Dposition:
    {
      unsigned long waitStarted = millis();
      const unsigned long maxWait = 120000; // Timeout after 120 seconds

      console.println(F("Waiting for valid dateTime and 3D position..."));

      bool keepGoing = true;
      
      while ((keepGoing) && (millis() < (waitStarted + maxWait)))
      {
        if (mySwarm.getDateTime(&dateTime) == SWARM_M138_SUCCESS) // Get the date and time from the modem
          if (mySwarm.getGpsFixQuality(&fix) == SWARM_M138_SUCCESS) // Request the most recent geospatial information
            if ((dateTime.valid) && ((fix.fix_type == SWARM_M138_GPS_FIX_TYPE_G3) || (fix.fix_type == SWARM_M138_GPS_FIX_TYPE_D3) || (fix.fix_type == SWARM_M138_GPS_FIX_TYPE_RK)))
              keepGoing = false;
        mySwarm.checkUnsolicitedMsg();
        delay(1000);
      }

      if (!keepGoing)
      {
        console.println(F("Modem has valid dateTime and 3D position"));
        loopState = queueMessage; // Move on
      }
      else
      {
        console.println(F("Modem does not have valid dateTime and/or 3D position. Trying again..."));
        delay(2000);
        loopState = startUp; // Repeat startUp
      }      
    }
    break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // queueMessage
    //
    // Queue a message containing: dateTime, position, temperature
    
    case queueMessage:
    {
      char message[192]; // Messages can be up to 192 bytes

      // Some platforms do not support sprintf %02d correctly
      // So, assemble the dateTime in ISO 8601 format manually
      char isoTime[21]; // YYYY-MM-DDTHH:MM:SSZ + null
      char scratchpad[10];

      mySwarm.getDateTime(&dateTime); // Get the date and time from the modem

      sprintf(isoTime, "%d", dateTime.YYYY);
      strcat(isoTime, "-");
      if (dateTime.MM < 10) strcat(isoTime, "0");
      sprintf(scratchpad, "%d", dateTime.MM);
      strcat(isoTime, scratchpad);
      strcat(isoTime, "-");
      if (dateTime.DD < 10) strcat(isoTime, "0");
      sprintf(scratchpad, "%d", dateTime.DD);
      strcat(isoTime, scratchpad);
      strcat(isoTime, "T");
      if (dateTime.hh < 10) strcat(isoTime, "0");
      sprintf(scratchpad, "%d", dateTime.hh);
      strcat(isoTime, scratchpad);
      strcat(isoTime, ":");
      if (dateTime.mm < 10) strcat(isoTime, "0");
      sprintf(scratchpad, "%d", dateTime.mm);
      strcat(isoTime, scratchpad);
      strcat(isoTime, ":");
      if (dateTime.ss < 10) strcat(isoTime, "0");
      sprintf(scratchpad, "%d", dateTime.ss);
      strcat(isoTime, scratchpad);
      strcat(isoTime, "Z");

      strcpy(message, isoTime); // Copy the isoTime into message

      strcat(message, ",");
      
      if (dateTime.valid) // Add Valid / Invalid
        strcat(message, "V");
      else
        strcat(message, "I");

      strcat(message, ",");
      
      mySwarm.getGeospatialInfo(&pos); // Request the most recent geospatial information
      
      // Some platforms do not support sprintf %03.4f correctly
      // So, append (concatenate) lat, lon and alt manually
      float lat = pos.lat;
      sprintf(scratchpad, "%d", (int)lat);
      strcat(message, scratchpad);
      strcat(message, ".");
      if (lat < 0.0) lat *= -1.0; // Make lat +ve
      lat -= (float)((int)lat); // Subtract the whole degrees
      for (int dp = 0; dp < 4; dp++) // The modem only provides 4 decimal places
      {
        lat *= 10.0;
        sprintf(scratchpad, "%d", (int)lat);
        strcat(message, scratchpad);
        lat -= (float)((int)lat);
      }
      strcat(message, ",");
      
      float lon = pos.lon;
      sprintf(scratchpad, "%d", (int)lon);
      strcat(message, scratchpad);
      strcat(message, ".");
      if (lon < 0.0) lon *= -1.0; // Make lon +ve
      lon -= (float)((int)lon); // Subtract the whole degrees
      for (int dp = 0; dp < 4; dp++) // The modem only provides 4 decimal places
      {
        lon *= 10.0;
        sprintf(scratchpad, "%d", (int)lon);
        strcat(message, scratchpad);
        lon -= (float)((int)lon);
      }
      strcat(message, ",");
      
      sprintf(scratchpad, "%d", (int)pos.alt);
      strcat(message, scratchpad);
      strcat(message, ",");

      // Append the temperature
      float degC;
      mySwarm.getTemperature(&degC);
      sprintf(scratchpad, "%d", (int)degC);
      strcat(message, scratchpad);
      strcat(message, ".");
      if (degC < 0.0) degC *= -1.0; // Make degC +ve
      degC -= (float)((int)degC); // Subtract the whole degrees
      for (int dp = 0; dp < 2; dp++) // Add 2 decimal places
      {
        degC *= 10.0;
        sprintf(scratchpad, "%d", (int)degC);
        strcat(message, scratchpad);
        degC -= (float)((int)degC);
      }

      // Print the message
      console.print(F("Message is: "));
      console.println(message);

      // Add it to the queue
      uint64_t msgID;
      Swarm_M138_Error_e err = mySwarm.transmitText(message, &msgID);

      if (err == SWARM_M138_SUCCESS)
      {
        console.print(F("Message has been added to the transmit queue. msg_id is: "));
        serialPrintUint64_t(msgID);
        console.println();
      }
      else
      {
        console.println(F("Message could not be added to the transmit queue. Maybe the queue is full?"));
      }

      loopState = waitForTx; // Move on      
    }
    break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // waitForTx
    //
    // Wait until the message has been transmitted
    // If we are waiting or more than an hour, queue another message
    
    case waitForTx:
    {
      uint16_t unsent;
      mySwarm.getUnsentMessageCount(&unsent);

      console.print(F("The queue contains "));
      console.print(unsent);
      if (unsent == 1)
        console.println(F(" unsent message"));
      else
        console.println(F(" unsent messages"));
        
      console.println(F("Waiting for the queue to empty"));
      
      unsigned long waitStarted = millis();
      unsigned long maxWait = ((unsigned long)txIntervalMins) * 60 * 1000; // Convert txIntervalMins to millis
      unsigned long waitUntil = waitStarted + maxWait;
      bool rollOver = waitUntil < waitStarted; // Check if millis will roll-over while we are waiting

      bool keepGoing = true;
      while (keepGoing)
      {
        mySwarm.getUnsentMessageCount(&unsent); // Check if the queue is empty
        
        if (unsent == 0)
          keepGoing = false;
        else
        {
          if (!rollOver) // Check if timeout has been reached
            keepGoing = millis() < waitUntil;
          else
            keepGoing = (millis() > waitStarted) || (millis() < waitUntil);
        }

        mySwarm.checkUnsolicitedMsg();
        delay(500);
      }

      if (unsent == 0)
      {
        console.println(F("The queue is empty. All messages have been sent"));
        loopState = goToSleep; // Move on
      }
      else
      {
        console.println(F("Timeout has expired. Adding a new message"));
        loopState = queueMessage;
      }        
    }
    break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // goToSleep
    //
    // Get ready to put the modem to sleep until it is time for the next transmit
    
    case goToSleep:
    {
      Swarm_M138_DateTimeData_t dateTimeNow;
      mySwarm.getDateTime(&dateTimeNow); // Get the date and time from the modem

      uint32_t epochAtQueueMessage = convertDateTimeToEpoch(&dateTime); // What was the time when we added the last message to the queue?
      uint32_t epochNow = convertDateTimeToEpoch(&dateTimeNow);

      secsToSleep = epochAtQueueMessage + (((uint32_t)txIntervalMins) * 60); // Add the tx interval to the time when the message was queued
      if (secsToSleep > epochNow)
        secsToSleep -= epochNow; // Subtract epochNow
      else
        secsToSleep = 0; // Prevent secsToSleep from going -ve

      if (secsToSleep > 60) // Don't sleep if there is less than one minute to the next transmit
      {
        console.print(F("Going to sleep for "));
        console.print(secsToSleep);
        console.println(F(" seconds"));

        loopState = sleepUntil;
      }
      else
      {
        console.print(F("Waiting for "));
        console.print(secsToSleep);
        console.println(F(" seconds"));

        delay(secsToSleep * 1000);
        loopState = queueMessage; // Move on
      }
    }
    break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // sleepUntil
    //
    // Put the modem to sleep until it is time for the next transmit
    
    case sleepUntil:
    {
      sleepWakeSeen = false; // Clear the $SL-seen flag
      gpio1InterruptSeen = false; // Clear the interrupt-seen flag

      mySwarm.sleepMode(secsToSleep);

      // If you can put your processor to sleep, do it here.
      // You can:
      //   sleep for secsToSleep seconds
      //   sleep and be woken by the GPIO1 interrupt
      //   sleep until the modem generates a $SL message

      
      // If you can not put your processor to sleep, then wait for an interrupt and/or a $SL message
      while ((!sleepWakeSeen) && (!gpio1InterruptSeen))
      {
        mySwarm.checkUnsolicitedMsg(); // Process the $SL message when it arrives
        delay(1000);
      }


      console.println(F("The modem is awake!"));

      if (gpio1InterruptSeen)
        console.println(F("GPIO1 interrupt seen!"));
      
      loopState = waitFor3Dposition; // Move on
    }
    break;
  }
} // /loop()

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

  console.print(fwd);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

uint32_t convertDateTimeToEpoch(Swarm_M138_DateTimeData_t *dateTime)
{

  const uint32_t SFE_DAYS_FROM_1970_TO_2020 = 18262; // Jan 1st 2020 Epoch = 1577836800 seconds
  const uint16_t SFE_DAYS_SINCE_2020[80] =
  {
    0, 366, 731, 1096, 1461, 1827, 2192, 2557, 2922, 3288,
    3653, 4018, 4383, 4749, 5114, 5479, 5844, 6210, 6575, 6940,
    7305, 7671, 8036, 8401, 8766, 9132, 9497, 9862, 10227, 10593,
    10958, 11323, 11688, 12054, 12419, 12784, 13149, 13515, 13880, 14245,
    14610, 14976, 15341, 15706, 16071, 16437, 16802, 17167, 17532, 17898,
    18263, 18628, 18993, 19359, 19724, 20089, 20454, 20820, 21185, 21550,
    21915, 22281, 22646, 23011, 23376, 23742, 24107, 24472, 24837, 25203,
    25568, 25933, 26298, 26664, 27029, 27394, 27759, 28125, 28490, 28855
  };
  const uint16_t SFE_DAYS_SINCE_MONTH[2][12] =
  {
    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}, // Leap Year (Year % 4 == 0)
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}  // Normal Year
  };

  uint32_t t = SFE_DAYS_FROM_1970_TO_2020;                                                // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_DAYS_SINCE_2020[dateTime->YYYY - 2020];                              // Add on the number of days since 2020
  t += (uint32_t)SFE_DAYS_SINCE_MONTH[dateTime->YYYY % 4 == 0 ? 0 : 1][dateTime->MM - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)dateTime->DD - 1;                                                        // Add on the number of days since the 1st of the month
  t *= 24;                                                                                // Convert to hours
  t += (uint32_t)dateTime->hh;                                                            // Add on the hour
  t *= 60;                                                                                // Convert to minutes
  t += (uint32_t)dateTime->mm;                                                            // Add on the minute
  t *= 60;                                                                                // Convert to seconds
  t += (uint32_t)dateTime->ss;                                                            // Add on the second

  return (t);
}
