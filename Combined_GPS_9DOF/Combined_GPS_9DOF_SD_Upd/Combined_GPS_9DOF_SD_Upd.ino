//TODO: Record quaternions, at a higher frequency than 1 Hz
// TODO: Move GPS from pin 0 and 1 to separate pins
//TODO Remove print "Update milli after SDSYNC_INTERVAL

//TODO: Do we need wire.h? wire is I2C
#include <Wire.h>
// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
// #include <Madgwick.h>
//TODO: Which one to use. Might be more accurate, but requires more calculations
// setup RTC Real Time Clock
//TODO: RTC might need wire.h but not sure
#include "RTClib.h"
RTC_PCF8523 rtc;

// TODO Remove all debug tests e.g GPSECHO in final code or use #if

// Note: This sketch is a WORK IN PROGRESS
// Combination of GPS_Parsing sketch and AHRS_fusion_usb sketch

// Connect the GPS Power pin to 3.3V on M0 (5V on uno)
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// Connect the 9DOF Power pin to GPS power pin
// Connect the 9DOF Ground pin to Arduino ground
// Connect the 9DOF SCL pin to Arduino SCL pin (I2C)
// Connect the 9DOF SDA pin to Arduino SDA pin (I2C)

// Adapt to M0 core
// Adjust serial.print
//ToDo: How if ARDUINO_ARCH_SAMD set?
// #if defined (ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, required for SAMD core, remove line below if using programming port to program the zero
#define Serial SerialUSB
// #endif

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  200 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SDSYNC_INTERVAL 200 // mills between calls to flush() - to write data to the card
uint32_t sdSyncTime = 0; // time of last sync()


//Debug flag to print all outputs in addition to writing to SD Card
#define ECHO_TO_SERIAL false
// Flag to wait for serial input in setup (true will wait). Needs to be true if ECHO_TO_SERIAL is true
#define WAIT_TO_START false // Wait for serial input in setup()
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY true
/* set to true to log debug info to SD */
#define DEBUG_TO_SD true
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false


// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// setup SD Card
//TODO: Initially open static named File
//TODO: Next open RTC named File
//TODO: Last, initialize GPS, wait for GPS Fix to get date as filename
File logFile;
// change this to match your SD shield or module;
//     Arduino Ethernet shield: pin 4
//     Adafruit SD shields and modules: pin 10
//     Sparkfun SD shield: pin 8
const int chipSelect = 10;


// Setup the GPS
// Updte code to listen to the GPS via polling. Use with M0 which has Hardware serial and no interrupt
#include <Adafruit_GPS.h>
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
// Create GPS instance
Adafruit_GPS GPS(&GPSSerial);
boolean usingInterrupt = false;
int GPSFix = 0;
char c;

// SAMD M0 core does not use interrupt (no available) but polling instead
// For Arduino Uno, this keeps track of whether we're using the interrupt
// off by default!
#if !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266) && !defined(ARDUINO_ARCH_STM32F2)
// boolean usingInterrupt = false;
// void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
#endif

uint32_t timer = millis();

// #if !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266) && !defined(ARDUINO_ARCH_STM32F2)
// DEBUG M0: commented out as already called in Adafruit_GPS.cpp...
// #include <SoftwareSerial.h>
 // If you're using the Adafruit GPS shield, change
 // SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
 // and make sure the switch is set to SoftSerial

 // If using software serial, keep this line enabled
 // (you can change the pin numbers to match your wiring):
//DEBUG M0: commented out as not using SoftwareSerial...
// SoftwareSerial mySerial(3, 2);
// #endif

// setup giros
// Define 9DOF Library and parameters
#define ST_LSM303DLHC_L3GD20        (0)
#define ST_LSM9DS1                  (1)
#define NXP_FXOS8700_FXAS21002      (2)

// Define your target sensor(s) here based on the list above!
// #define AHRS_VARIANT    ST_LSM303DLHC_L3GD20
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002

#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#else
#error "AHRS_VARIANT undefined! Please select a target sensor combination!"
#endif



// Create 9DOF sensor instances.
#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 14.55F, -1.15F, 75.47F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.995,  -0.019,  -0.011 },
                                    {  0.019,  0.969,  0.022 },
                                    {  -0.011,  0.022,  1.039 } };

float mag_field_strength        = 46.38F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
//To use Madwick simply change the comments to filter
Mahony filter;
//Madgwick filter;

// Objects to read the 9DOF sensor
sensors_event_t gyro_event;
sensors_event_t accel_event;
sensors_event_t mag_event;

float roll;
float pitch;
float heading;

// how many milliseconds before recording the new set of quaternions
#define QUATREAD_INTERVAL 100 // mills between calls to record the quaternions into a variable that will be transfered to the file
uint32_t quatTimer = millis();
uint32_t quat1 = millis();
uint32_t quat2 = millis();

#define QUAT_PER_SEC 2
float tempqw, tempqx, tempqy, tempqz;
float qw1, qx1, qy1, qz1;
float qw2, qx2, qy2, qz2;
float qw[QUAT_PER_SEC], qx[QUAT_PER_SEC], qy[QUAT_PER_SEC], qz[QUAT_PER_SEC];
int quatIndex = 0;
#define gyro9DoF 20 // number of samples per second

// DEBUG Alternative to blink an error code. Combine the 2 functions. Use error code or str?
// void error(char *str)
// {
  // Serial.print("error: ");
  // Serial.println(str);
  // red LED indicates error
  // digitalWrite(redLEDpin, HIGH);
  // while(1);
// }

// blink out an error code
void error(uint8_t errno) {
/*
  if (SD.errorCode()) {
    putstring("SD error: ");
    Serial.print(card.errorCode(), HEX);
    Serial.print(',');
    Serial.println(card.errorData(), HEX);
  }
  */
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(redLEDpin, HIGH);
      delay(100);
      digitalWrite(redLEDpin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup()
{
// Important for SAMD core
//TODO Can we skip the Serial begin and setup if not using serial port: TO BE TESTED
// Wait for the Serial Monitor to open (comment out to run without Serial Monitor);
#if (WAIT_TO_START)
  while(!Serial);
  Serial.begin(115200);
#endif

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
#if (ECHO_TO_SERIAL)
  Serial.println("Initializing Adafruit GPS + AHRS-9DOF Fusion + SD Card Combined");
#endif

  // Initialize the GPS
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
//DEBUG01  GPSSerial.begin(9600);
//DEBUG01  while(!GPSSerial);

  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
  // Delay to give time for the data to get to the GPS
  delay(1000);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  // Uncomment to turn off antenna status stream
  // GPS.sendCommand("$PGCMD,33,0*6D");

  // Delay to give time for the data to get to the GPS
  delay(1000);

// Ask for firmware version
//DEBUG: Remove following line?
//  GPSSerial.println(PMTK_Q_RELEASE);

#if (ECHO_TO_SERIAL)
  Serial.println("Adafruit GPS_HardwareSerial_Parsing configuration done");
#endif

  // SAMD M0 core does not use interrupt (no available) but polling instead
  // If not M0 core, with interrupt available, uncomment useInterrupt
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  // useInterrupt(true);



  // Initialize the 9DOF sensors.
#if (ECHO_TO_SERIAL)
  Serial.println("Adafruit 9DOF configuration");
#endif

  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    //TODO use error function
#if (ECHO_TO_SERIAL)
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
 #endif
   while(1);
  }

#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
      //TODO use error function
#if (ECHO_TO_SERIAL)
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
#endif
    while(1);
  }
#else
  //TODO: does not go in this loop, as the type of 9DOF is known
  if (!accel.begin())
  {
    /* There was a problem detecting the accel ... check your connections */
    //TODO use error function
    Serial.println("Ooops, no accel detected ... Check your wiring!");
   while (1);
  }

  if (!mag.begin())
  {
    /* There was a problem detecting the mag ... check your connections */
    //TODO use error function
    Serial.println("Ooops, no mag detected ... Check your wiring!");
   while (1);
  }
#endif

  // Filter expects 70 (gyro9DoF) samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(gyro9DoF);

  // TODO: Why do we need this?
  delay(1000);
#if (ECHO_TO_SERIAL)
  Serial.println("Adafruit 9DOF configuration done");
#endif

  // Initialize the RTC to create filname for SD Card
  if (! rtc.begin()) {
      //TODO use error function
#if (ECHO_TO_SERIAL)
    Serial.println("Couldn't find RTC");
#endif
    while (1);
  }

  if (! rtc.initialized()) {
      //TODO use error function

#if (ECHO_TO_SERIAL)
    Serial.println("RTC is NOT running!");
#endif
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  // Initialie the SD Card
#if (ECHO_TO_SERIAL)
  Serial.print("Initializing SD card...");
#endif
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  // Using 10 for Arduino M0
   pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
//TODO use error function
#if (ECHO_TO_SERIAL)
    Serial.println("initialization failed!");
#endif
    return;
  }
#if (ECHO_TO_SERIAL)
  Serial.println("  initialization done.");
#endif

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  char filename[] = "00000000.CSV";

  // TODO Read date from GPS - when got a fix => Need to stay in setup until got a fix
  // TODO Get in loop only when all data are good quality
  DateTime now = rtc.now();

  filename[0] = (now.year()/10)%10 + '0'; //To get 3rd digit from year()
  filename[1] = now.year()%10 + '0'; //To get 4th digit from year()
  filename[2] = now.month()/10 + '0'; //To get 1st digit from month()
  filename[3] = now.month()%10 + '0'; //To get 2nd digit from month()
  filename[4] = now.day()/10 + '0'; //To get 1st digit from day()
  filename[5] = now.day()%10 + '0'; //To get 2nd digit from day()

  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logFile = SD.open(filename, FILE_WRITE);

#if (ECHO_TO_SERIAL)
        Serial.print(filename);
        Serial.println(" doesn't exist. ");
        Serial.print("Creating new file: ");
        Serial.println(filename);
#endif

      break;  // leave the loop!

    } else {
#if (ECHO_TO_SERIAL)
        Serial.print(filename);
        Serial.println(" exists.");
#endif
    }

  }

  // if the file opened okay, write to it:
   if (logFile) {

#if (ECHO_TO_SERIAL)
     Serial.print("SD Writing header to file: ");
     Serial.print(filename);
#endif
    // Write in first line of cvs file the headers of each column
    // Time (hours), Time(Minutes), Time(seconds.milliseconds),
    // Fix, quality
    // Location
// TODO write time as hrs:mins:sec rather than hrs, min, second
// TODO write date as day/month/year rather than day, month, year

     logFile.println("Time hrs, Min, Sec.millis, Date day, month, year, lat in deg, long in deg, speed (knots), Orientation head, pitch, roll, time quat1, qw1, qx1, qy1, qz1, time quat2, qw2, qx2, qy2, qz2");
    // push to the SD Card
     logFile.flush();

#if (ECHO_TO_SERIAL)
     Serial.println(" .... SD done.");
#endif
   } else {
     // if the file didn't open, print an error:
     //TODO use error function
#if (ECHO_TO_SERIAL)
     Serial.println("SD error opening file");
#endif
   }
#if (ECHO_TO_SERIAL)
   Serial.print("GPS.fix returns: ");
   Serial.println(GPS.fix);
#endif

   // clear the buffer, to only get full sentences
   clearGPS();

   // Wait for the GPS to get a fix as last step of setup, before moving to the loop
   // Bypass wait for fix to start recording on SD earlier during debug
// #if (LOG_FIXONLY)
if (LOG_FIXONLY) {
#if (ECHO_TO_SERIAL)
   Serial.println("IN LOG_FIXONLY");
#endif
// wait to get a fix before starting to record
   while(!GPS.fix) {
// DEBUG attempt to use the time waiting for the fix to do something else...
 //      int count;
 // query the gyro while waiting for a fix, to stabilize the AHRS filter
 //      for (count = 0; count < gyro9DoF; count++) {
 // read gyro and process data
 //         readGyro9DoF();
 //      }

       readGPS();
//DEBUG remove the clear to see if we get faster to a fix
       clearGPS();
#if (ECHO_TO_SERIAL)
   Serial.println("OUT LOG_FIXONLY");
#endif       
   }
 } //end of if LOG_FIXONLY

//#endif

//TODO wait fro about 2 mins for 9DoF to stabilize...
// Loop to measure that change between measures is less than x, in particular heading

//TODO Move the SD open file after the GSP has a fix, to use the right date to create the file name

} // End setup

// The following function is only used with a core with interrupt. M0 does not
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
//SIGNAL(TIMER0_COMPA_vect) {
  // char c = GPS.read();
  // if you want to debug, this is a good time to do it!
//#ifdef UDR0
  // if (GPSECHO)
    // if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
//#endif
//}

// The following function is only used with a core with interrupt. M0 does not
// void useInterrupt(boolean v) {
    // if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    // OCR0A = 0xAF;
    // TIMSK0 |= _BV(OCIE0A);
    // usingInterrupt = true;
  // } else {
    // do not call the interrupt function COMPA anymore
    // TIMSK0 &= ~_BV(OCIE0A);
    // usingInterrupt = false;
  // }
// }


void loop(void)
{

  // First read the GPS
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
// DEBUG: could uncomment following line to make code more generic
//  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    // char c = GPS.read();
    //DEBUG if you want to debug, this is a good time to do it!
    // if (GPSECHO)
      // if (c) Serial.print(c);
//  }

  int count;
// How quickly can we read the gyro?
  for (count = 0; count < gyro9DoF; count++) {
      // read gyro and process data
      readGyro9DoF();
//TODO write more gyro reads on SD not hard coded
      if (quatTimer > millis())  quatTimer = millis();

      // approximately every 500 milliseconds or so, print out the current quaternion
      if (millis() - quatTimer > QUATREAD_INTERVAL) {
          quatTimer = millis(); // reset the timer
          qw[quatIndex] = tempqw;
          qx[quatIndex] = tempqx;
          qy[quatIndex] = tempqy;
          qz[quatIndex] = tempqz;      
          quatIndex ++;
          if (quatIndex == QUAT_PER_SEC) quatIndex = 0; // reset the index of the matrix where we write the quaternions 
      }    
      if (count == 9) {
          quat1 = millis();
          qw1 = tempqw;
          qx1 = tempqx;
          qy1 = tempqy;
          qz1 = tempqz;
      }
  } // end of for (count)

// read GPS data - will wait for a full set of data, coming at 1Hz
  readGPS();
// when new GPS data has been received, read a new 9DOF data set, as readGPS runs at 1Hz only
  readGyro9DoF();
    quat2 = millis();
    qw2 = tempqw;
    qx2 = tempqx;
    qy2 = tempqy;
    qz2 = tempqz;  

//DEBUG
// #if (DEBUG_TO_SD)
//            logFile.println("Reading GPS after looping on 9DOF");
//            logFile.flush();
//#endif

  // Print combined GPS position + 9DOF
  // DEBUG Does this make sense, as we check that millis increment by 1000 at line 295
   // if millis() or timer wraps around after long period, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > LOG_INTERVAL) {
      timer = millis(); // reset the timer

      digitalWrite(greenLEDpin, HIGH);

      if (logFile) {
          logFile.print(GPS.hour, DEC); logFile.print(',');
          logFile.print(GPS.minute, DEC); logFile.print(',');
          logFile.print(GPS.seconds, DEC); logFile.print('.');
          logFile.print(GPS.milliseconds); logFile.print(',');
          logFile.print(GPS.day, DEC); logFile.print(',');
          logFile.print(GPS.month, DEC); logFile.print(',');
          logFile.print(GPS.year, DEC); logFile.print(',');

#if (ECHO_TO_SERIAL)

          Serial.print("\nTime: ");
          Serial.print(GPS.hour, DEC); Serial.print(':');
          Serial.print(GPS.minute, DEC); Serial.print(':');
          Serial.print(GPS.seconds, DEC); Serial.print('.');
          Serial.print(GPS.milliseconds);
          Serial.print("\tDate: ");
          Serial.print(GPS.day, DEC); Serial.print('/');
          Serial.print(GPS.month, DEC); Serial.print("/20");
          Serial.println(GPS.year, DEC);
          // DEBUG: can comment the next 2 lines after DEBUG
          //    GPSFix = (int)GPS.fix;
          //    Serial.print("Fix sep. var.: "); Serial.print(GPSFix);
          Serial.print("Fix: "); Serial.print((int)GPS.fix);
          Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
#endif //ECHO_TO_SERIAL

          logFile.print(GPS.latitudeDegrees, 4); logFile.print(',');
          logFile.print(GPS.longitudeDegrees, 4);  logFile.print(',');
          logFile.print(GPS.speed); logFile.print(',');

#if (ECHO_TO_SERIAL)
          // Serial.print("Location: ");
          // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          // Serial.print(", ");
          // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Location (in deg for G. Maps): ");
          Serial.print(GPS.latitudeDegrees, 4);
          Serial.print(", ");
          Serial.print(GPS.longitudeDegrees, 4);
          Serial.print("  -  Speed (knots): "); Serial.println(GPS.speed);
          // Serial.print("Angle: "); Serial.println(GPS.angle);
          // Serial.print("Altitude: "); Serial.println(GPS.altitude);
          //DEBUG: Could comment following line when not debugging
          // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
#endif //ECHO_TO_SERIAL


          // SLog Euler angles
          logFile.print(heading); logFile.print(',');
          logFile.print(pitch); logFile.print(',');
          logFile.print(roll); logFile.print(',');
          
          // SLog quaternions
          logFile.print(quat1); logFile.print(',');
          logFile.print(qw1); logFile.print(',');
          logFile.print(qx1); logFile.print(',');
          logFile.print(qy1); logFile.print(',');
          logFile.print(qz1); logFile.print(',');

          logFile.print(quat2); logFile.print(',');
          logFile.print(qw2); logFile.print(',');
          logFile.print(qx2); logFile.print(',');
          logFile.print(qy2); logFile.print(',');
          logFile.print(qz2);          
          logFile.println("");


#if (ECHO_TO_SERIAL)
          // Serial.print(millis());
          Serial.print(" - Orientation - Head, Pitch, Roll:");
          Serial.print(heading);
          Serial.print(" ");
          Serial.print(pitch);
          Serial.print(" ");
          Serial.println(roll);
#endif //ECHO_TO_SERIAL

          digitalWrite(greenLEDpin, LOW);

    // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
    // which uses a bunch of power and takes time
          if ((millis() - sdSyncTime) > SDSYNC_INTERVAL) {
//#if (ECHO_TO_SERIAL)
//              Serial.println("write to SD after SDSYNC_INTERVAL");
//#endif
              sdSyncTime = millis();

              // blink LED to show we are syncing data to the card & updating FAT!
              digitalWrite(redLEDpin, HIGH);
              logFile.flush();
              digitalWrite(redLEDpin, LOW);

          } // end of write to SD

      } // End if(logFile)
      else {
//TODO use error function
#if (ECHO_TO_SERIAL)
          Serial.print("SD card file not opened");
#endif
      }

  } // End of millis timer loop
// DEBUG Why do we need this? This was in the 9DOF code
  delay(10);
} // end loop

void readGPS() {
    while(!GPS.newNMEAreceived()) {
          c = GPS.read();
    }
    // DEUBG: Old implementation below, not chekcing for full sentences
    // if a sentence is received, we can check the checksum, parse it...
     //if (GPS.newNMEAreceived()) {
       // a tricky thing here is if we print the NMEA sentence, or data
       // we end up not listening and catching other sentences!
       // so be very wary if using OUTPUT_ALLDATA and trying to print out data

       if (GPSECHO) {
#if (ECHO_TO_SERIAL)
          Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
#endif

#if (DEBUG_TO_SD)
            logFile.println(GPS.lastNMEA());
            logFile.flush();
#endif
       }

       if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
   //TODO use error function
#if (ECHO_TO_SERIAL)
        Serial.println("GPS sentence cannot be parsed");
#endif
#if (DEBUG_TO_SD)
            logFile.println("GPS sentence cannot be parsed");
            logFile.flush();
#endif

         return;  // we can fail to parse a sentence in which case we should just wait for another
     }

}

void clearGPS() {  //Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
#if (ECHO_TO_SERIAL)
        Serial.println("entering clearGPS");
#endif
    while(!GPS.newNMEAreceived()) {
        c=GPS.read();
    }
    // DEBUG do we need to parse, since we only want to empyt the buffer
#if (ECHO_TO_SERIAL)
        Serial.println("read the first newNMEAreceived");
#endif
    GPS.parse(GPS.lastNMEA());
    while(!GPS.newNMEAreceived()) {
        c=GPS.read();
    }
#if (ECHO_TO_SERIAL)
        Serial.println("read the second newNMEAreceived");
#endif
    // DEBUG do we need to parse, since we only want to empyt the buffer
    GPS.parse(GPS.lastNMEA());

#if (ECHO_TO_SERIAL)
        Serial.println("exiting clearGPS");
#endif
} // end clearGPS


void readGyro9DoF() {

  // Get new data samples
  gyro.getEvent(&gyro_event);
#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  accelmag.getEvent(&accel_event, &mag_event);
#else
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
#endif

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
  // Using Euler angles as easier to understand
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  
  // Record the orientation filter output in quaternions
  // avoids the gimbal lock problem with Euler angles when you get
  // close to 180 degrees (causing the model to rotate or flip, etc.)
  filter.getQuaternion(&tempqw, &tempqx, &tempqy, &tempqz);


}

