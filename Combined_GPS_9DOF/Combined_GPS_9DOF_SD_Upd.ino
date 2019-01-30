//TODO: Do we need wire.h?
#include <Wire.h>
// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
//TODO: Which one to use. Might be more accurate, but requires more calculations
// #include <Madgwick.h>


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

// setup RTC Real Time Clock
//TODO: RTC might need wire.h but not sure
#include "RTClib.h"
RTC_PCF8523 rtc;

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

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false

//Debug flag to print all outputs in addition to writing to SD Card
#define ECHO_TO_SERIAL false

// SAMD M0 core does not use interrupt (no available) but polling instead
// For Arduino Uno, this keeps track of whether we're using the interrupt
// off by default!
#if !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266) && !defined(ARDUINO_ARCH_STM32F2)
// boolean usingInterrupt = false;
// void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
#endif

uint32_t timer = millis();

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
float mag_offsets[3]            = { 13.37F, -4.91F, 79.91F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  1.012,  0.003,  0.004 },
                                    {  0.003,  0.963,  0.004 },
                                    {  0.003,  0.005,  1.027 } };

float mag_field_strength        = 48.87F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
//TODO: Can we use Madwick on M0? How?
Mahony filter;
//Madgwick filter;

void setup()
{
  // Important for SAMD core
  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor);
  while(!Serial);
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println(F("Adafruit GPS + AHRS-9DOF Fusion + SD Card Combined"));

  // Initialize the GPS
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

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
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  Serial.println("Adafruit GPS_HardwareSerial_Parsing configuration done");

  // SAMD M0 core does not use interrupt (no available) but polling instead
  // If not M0 core, with interrupt available, uncomment useInterrupt
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  // useInterrupt(true);



  // Initialize the 9DOF sensors.
  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
#else
  if (!accel.begin())
  {
    /* There was a problem detecting the accel ... check your connections */
    Serial.println("Ooops, no accel detected ... Check your wiring!");
    while (1);
  }

  if (!mag.begin())
  {
    /* There was a problem detecting the mag ... check your connections */
    Serial.println("Ooops, no mag detected ... Check your wiring!");
    while (1);
  }
#endif

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(10);

  // TODO: Why do we need this?
  delay(1000);


  // Initialize the RTC to create filname for SD Card
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  // Initialie the SD Card
  //TODO: use date as filename
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
   pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");


  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  char filename[] = "00000000.CSV";

  DateTime now = rtc.now();

  filename[0] = (now.year()/1000)%10 + '0'; //To get 1st digit from year()
  filename[1] = (now.year()/100)%10 + '0'; //To get 2nd digit from year()
  filename[2] = (now.year()/10)%10 + '0'; //To get 3rd digit from year()
  filename[3] = now.year()%10 + '0'; //To get 4th digit from year()
  filename[4] = now.month()/10 + '0'; //To get 1st digit from month()
  filename[5] = now.month()%10 + '0'; //To get 2nd digit from month()
  filename[6] = now.day()/10 + '0'; //To get 1st digit from day()
  filename[7] = now.day()%10 + '0'; //To get 2nd digit from day()
  Serial.println(filename);

   //Check file name exist?
  if (SD.exists(filename)) {
    Serial.print(filename);
    Serial.println(" exists.");
  }
  else {
    Serial.print(filename);
    Serial.println("doesn't exist. ");
    Serial.print("Creating new file: ");
    Serial.println(filename);
    // logFile = SD.open(filename, FILE_WRITE);
    // logFile.close();
  }

  //Serial.println("file: ");
  //Serial.println(filename);

  logFile = SD.open(filename, FILE_WRITE);
  // if the file opened okay, write to it:
   if (logFile) {
     Serial.print("SD Writing header to file: ");
     Serial.print(filename);

    // Write in first line of cvs file the headers of each column
    // Time (hours), Time(Minutes), Time(seconds.milliseconds),
    // Fix, quality
    // Location

     logFile.println("Time hrs, Min, Sec.millis, Date day, month, year, lat in deg, long in deg, speed (knots), Orientation head, pitch, roll");
    // close the file:
    // logFile.close();
     Serial.println(" .... SD done.");
   } else {
     // if the file didn't open, print an error:
     Serial.println("SD error opening file");
   }


}

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
  int GPSFix = 0;

  // First read the GPS
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
// DEBUG: could uncomment following line to make code more generic
//  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
//  }
  // if a sentence is received, we can check the checksum, parse it...
   if (GPS.newNMEAreceived()) {
     // a tricky thing here is if we print the NMEA sentence, or data
     // we end up not listening and catching other sentences!
     // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
     // DEBUG: uncomment following line to print more data
     // Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

     if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
       return;  // we can fail to parse a sentence in which case we should just wait for another
   }

  // Read the 9DOF sensor
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

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

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();

  // Print combined GPS position + 9DOF
  // DEBUG Does this make sense, as we check that millis increment by 1000 at line 295
   // if millis() or timer wraps around, we'll just reset it
   if (timer > millis())  timer = millis();

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
//    logFile.println("Time hrs, Min, Sec.millis, Date day, month, year, lat in deg, long in deg, speed (knots), Orientation head, pitch, roll");

    logFile.print(GPS.hour, DEC); logFile.print(',');
    logFile.print(GPS.minute, DEC); logFile.print(',');
    logFile.print(GPS.seconds, DEC); logFile.print('.');
    logFile.print(GPS.milliseconds); logFile.print(',');
    logFile.print(GPS.day, DEC); logFile.print(',');
    logFile.print(GPS.month, DEC); logFile.print(',');
    logFile.print(GPS.year, DEC); logFile.print(',');
    // DEBUG: can comment the next 2 lines after DEBUG
//    GPSFix = (int)GPS.fix;
//    Serial.print("Fix sep. var.: "); Serial.print(GPSFix);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    if (GPS.fix) {
//    if (GPSFix) {
      // Serial.print("Location: ");
      // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      // Serial.print(", ");
      // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Location (in deg for G. Maps): ");
      logFile.print(GPS.latitudeDegrees, 4); logFile.print(',');
      logFile.print(GPS.longitudeDegrees, 4);  logFile.print(',');
      logFile.print(GPS.speed); logFile.print(',');
      // Serial.print("Angle: "); Serial.println(GPS.angle);
      // Serial.print("Altitude: "); Serial.println(GPS.altitude);
      //DEBUG: Could comment following line when not debugging
      // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }

      // Serial.print(millis());
      logFile.print(heading); logFile.print(',');
      logFile.print(pitch); logFile.print(',');
      logFile.println(roll);

  }

//DEBUG should be #ifdef to avoid unnecessary code
if (ECHO_TO_SERIAL) {
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

    if (GPS.fix) {
//    if (GPSFix) {
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
    }

      // Serial.print(millis());
      Serial.print(" - Orientation - Head, Pitch, Roll:");
      Serial.print(heading);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(roll);

}
// DEBUG Why do we need this? This was in the 9DOF code
  delay(10);
}
