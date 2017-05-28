#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
  

File logfile;
int const smallPM1 = 13;
int const largePM1 = 12;
long const sampleRate = 20;
long measurementCount = 0;
long smallPM1Count = 0;
long largePM1Count = 0;
long priorSampleTime = 0;
double smallPM1percentRunning;
double largePM1percentRunning;

#define chipSelect 10
#define ledPin 3             // tis conflicts when SD library is in use
#define GPSECHO  false       // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console, Set to 'true' if you want to debug and listen to the raw GPS sentences
#define LOG_FIXONLY false     //set to true to only log to SD when GPS has a fix, for debugging, keep it false

// GPS Init
SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);

// Light Init
Adafruit_TSL2561_Unified lght_snsr = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);        // The address will be different depending on whether you leave the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively


void setup() {
  Serial.begin(9600);
  Serial.println(F("\r\nAir Quality Sensing Package Initializing"));
  pinMode(ledPin, OUTPUT);

  pinMode(10, OUTPUT);    // make sure that the default chip select pin is set to output, even if you don't use it:
  pinMode (smallPM1, INPUT);
  pinMode (largePM1, INPUT);
  
  sdCardInitialization();

  Serial.println(F("Initializing Light sensor..."));
  if(!lght_snsr.begin())
  {
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } else {
    Serial.println(F("  Light initialized"));
  }
 
  displayLightSensorDetails();
  configureLightSensor();
  
  Serial.println(F("Initializing GPS..."));
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);   // uncomment this line to turn on only the "minimum recommended" data; For logging data, we don't suggest using anything but either RMC only or RMC+GGA to keep the log files at a reasonable size
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // Set the update rate; 1 or 5 Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  // GPS.sendCommand(PGCMD_NOANTENNA);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Ask for firmware version
  gps_Serial.println(PMTK_Q_RELEASE);

  Serial.println(F("Ready!"));
}


int lastMillis = 0;
int loopCount = 1;

void loop(void) 
{ 
  gps_Serial.listen();
  char c = GPS.read();

  if (GPSECHO) {
     if (c)   Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data we end up not listening and catching other sentences! so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    GPS.lastNMEA();
    
    if (!GPS.parse(GPS.lastNMEA()))  { // this also sets the newNMEAreceived() flag to false
      //Serial.println(F("Failed to parse GPS data"));
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    
    // Sentence parsed! 
    //Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
        Serial.println("No Fix");
        return;
    }

    //Serial.println("Write to File");
    
    logfile.println(F("#####"));  // new data marker
    printHeader();
    
    char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))  {  //write the string to the SD file
      Serial.println(F("error with writing to SD")); 
      //error(4);
    }
    else {
      logfile.println(); 
    }
    logGPSData();
    logLight();
    logCO();
    samplePMDetectors();         //sample particulate detectors for 2 seconds and update running Averages
    printRunningPMDataToSerial(); //print percentages to Serial
    logRunningPMDataToSerial();
    
    logfile.flush();
  }
}


void sdCardInitialization(void)
{
  // SD Card Init
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD Card init. failed!"));
    while(1);
  } else {
  Serial.println(F("SD Card init successful!"));
  }
  
  // File Init
  char filename[15];
  sprintf(filename, "/LOG00.txt");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    if (! SD.exists(filename)) {          // create if does not exist, do not open existing, write, sync after write
      break;
    }
  }

  Serial.print(F("creating file: "));
  Serial.println(filename);
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print(F("Couldnt create ")); Serial.println(filename);
    while(1);
  }
  Serial.print(F("Writing to ")); Serial.println(filename);
}

    
void displayLightSensorDetails(void)
{
  sensor_t sensor;
  lght_snsr.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void configureLightSensor(void)         //Configures the gain and integration time for the TSL2561
{
  // lght_snsr.setGain(TSL2561_GAIN_1X);      // No gain ... use in bright light to avoid sensor saturation
  // lght_snsr.setGain(TSL2561_GAIN_16X);     // 16x gain ... use in low light to boost sensitivity
  lght_snsr.enableAutoRange(true);            // Auto-gain ... switches automatically between 1x and 16x
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  lght_snsr.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // light_snsr.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // light_snsr.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
 
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}
  
   
void logGPSData() {
  logfile.print(F("GPS1,20"));
  logfile.print(GPS.year, DEC); logfile.print(F(","));
  logfile.print(GPS.month, DEC); logfile.print(F(","));
  logfile.print(GPS.day, DEC);
  logfile.print(F(","));
  logfile.print(GPS.hour, DEC); logfile.print(F(","));
  logfile.print(GPS.minute, DEC); logfile.print(F(","));
  logfile.print(GPS.seconds, DEC); logfile.print(F(","));
  logfile.print(GPS.milliseconds);
  logfile.print(F(","));
  logfile.print(F("fix_")); logfile.print((int)GPS.fix);
  logfile.print(F(","));
  logfile.print(F("qual_")); logfile.print((int)GPS.fixquality); 
  logfile.println();
  if (GPS.fix) {
    logfile.print(F("GPS2,"));
    logfile.print(GPS.latitude, 6); logfile.print(F(",")); logfile.print(GPS.lat);
    logfile.print(F(",")); 
    logfile.print(GPS.longitude, 6); logfile.print(F(",")); logfile.print(GPS.lon);
    logfile.print(F(",")); logfile.print(GPS.altitude);
    logfile.print(F(",")); logfile.print(GPS.speed);
    logfile.print(F(",")); logfile.print((int)GPS.satellites);
    logfile.println();
  }
}   
   
    
void logLight(void)
{ 
  sensors_event_t event;      //Get a new sensor event
  lght_snsr.getEvent(&event);
 
  // Display the results (light is measured in lux)
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");
    logfile.print(F("Light Sensor, "));
    logfile.print(event.light);
    logfile.print(F(" lux"));
    logfile.println();
    } else {
      Serial.println("Sensor overload");      // If event.light = 0 lux the sensor is probably saturated and no reliable data could be generated!
      logfile.println(F("Light Sensor Overloaded"));
    }
}


void logCO(void)
{
  int reading = analogRead(A0);
  Serial.println("CO Sensor, "); Serial.println(reading);
  logfile.print(F("CO Sensor, "));
  logfile.print(reading);
  analogWrite(ledPin, reading);
  logfile.println();
  delay(10);
}


void samplePMDetectors() {
  for (int i = 0; i < 100; i++) {
    while (millis() - priorSampleTime < sampleRate) {
    }
    priorSampleTime = millis();
    measurementCount += 1;
    if (digitalRead(smallPM1) == 0) {
      smallPM1Count += 1;
    }
    if (digitalRead(largePM1) == 0) {
      largePM1Count += 1;
    }
  }
  //calculate running PM percentages
  smallPM1percentRunning = 100.0 * smallPM1Count / measurementCount;
  largePM1percentRunning = 100.0 * largePM1Count / measurementCount;
}


void timestampSerial() {
  Serial.print("Milliseconds since the program started: ");
  Serial.println(millis());
}

void printRunningPMDataToSerial() {
  Serial.println("Particulate Matter Data");
  Serial.print("Measurement Count:  ");
  Serial.println(measurementCount);
  Serial.print("Small PM detector 1: ");
  Serial.println(smallPM1percentRunning);
  Serial.print("Large PM detector 1: ");
  Serial.println(largePM1percentRunning);
  Serial.println();
}


void logRunningPMDataToSerial() {
  logfile.println("Particulate Matter Data");
  logfile.print("Measurement Count:  ");
  logfile.println(measurementCount);
  logfile.print("Small PM detector 1: ");
  logfile.println(smallPM1percentRunning);
  logfile.print("Large PM detector 1: ");
  logfile.println(largePM1percentRunning);
  logfile.println();
}


void printHeader() {
  Serial.print(F("##### Sample "));   // new data marker
  Serial.print(loopCount++);
  Serial.print(F(" "));
  Serial.print(millis());
  Serial.println(F(" ########################################")); 
}
