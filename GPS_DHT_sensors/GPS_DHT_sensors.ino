#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"

// DHT data structure
struct _dhtData {
  boolean valid;
  float   humidity;
  float   temp_c;     // temperature in celsius
  float   temp_f;     // temperature in fahreheit
  float   heatIndex;  // heat index
};

_dhtData dhtData;
File logfile;

#define chipSelect 10
#define ledPin 3
#define DHTTYPE DHT22        // DHT 22  (AM2302)
#define DHTPIN 2
#define GPSECHO  false       // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console, Set to 'true' if you want to debug and listen to the raw GPS sentences
#define LOG_FIXONLY true     //set to true to only log to SD when GPS has a fix, for debugging, keep it false

// GPS Init
SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);

// DHT Init
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);

//  sdCardInitialization();
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
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
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

  Serial.println(F("Initializing DHT sensor..."));
  dht.begin();
  Serial.println(F("  DHT initialized"));

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

int loopCount = 1;
int dhtSkip = false;

void loop() {
//  Serial.println(F("in loop!!!"));
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

    printGPSData();
    logGPSData();
    printDHT();
    logDHT();
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
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
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


void printGPSData() {
  Serial.print(F("GPS: "));
  Serial.print(GPS.month, DEC); Serial.print(F("/"));
  Serial.print(GPS.day, DEC); Serial.print(F("/20"));
  Serial.print(GPS.year, DEC);
  Serial.print(F(" "));
  Serial.print(GPS.hour, DEC); Serial.print(F(":"));
  Serial.print(GPS.minute, DEC); Serial.print(F(":"));
  Serial.print(GPS.seconds, DEC); Serial.print(F("."));
  Serial.print(GPS.milliseconds);
  Serial.print(F(" "));
  Serial.print(F("Fix: ")); Serial.print((int)GPS.fix);
  Serial.print(F(" quality: ")); Serial.print((int)GPS.fixquality);
  Serial.println();
  if (GPS.fix) {
    Serial.print(F("LOC: "));
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(F(", "));
    Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
    Serial.print(F(" Alt: ")); Serial.print(GPS.altitude);
    Serial.print(F(" Spd: ")); Serial.print(GPS.speed);
    Serial.print(F(" Sats: ")); Serial.print((int)GPS.satellites);
    Serial.println();
  }
}

void logGPSData() {
  logfile.print(GPS.month, DEC); logfile.print(F("/"));
  logfile.print(GPS.day, DEC); logfile.print(F("/"));
  logfile.print(F("20"));
  logfile.print(GPS.year, DEC);
  logfile.print(F(","));
  logfile.print(GPS.hour, DEC); logfile.print(F(":"));
  logfile.print(GPS.minute, DEC); logfile.print(F(":"));
  logfile.print(GPS.seconds, DEC); logfile.print(F(":"));
  logfile.print(GPS.milliseconds);
  logfile.print(F(","));
  logfile.print(F("fix_")); logfile.print((int)GPS.fix);
  logfile.print(F(","));
  logfile.print(F("qual_")); logfile.print((int)GPS.fixquality);
  logfile.print(F(","));
  if (GPS.fix) {
    logfile.print(gpsConverter(GPS.latitude), 6); logfile.print(F(",")); logfile.print(GPS.lat);
    logfile.print(F(","));
    logfile.print(gpsConverter(GPS.longitude), 6); logfile.print(F(",")); logfile.print(GPS.lon);
    logfile.print(F(",")); logfile.print(GPS.altitude);
    logfile.print(F(",")); logfile.print(GPS.speed);
    logfile.print(F(",")); logfile.print((int)GPS.satellites);
    logfile.println();
  }
}

float gpsConverter(float gps)
{
  int gps_min = (int)(gps/100);
  float gps_sec = fmod(gps, 100) / 60;
  float gps_dec = gps_min + gps_sec;
  return gps_dec;
}


void printHeader() {
  Serial.print(F("##### Sample "));   // new data marker
  Serial.print(loopCount++);
  Serial.print(F(" "));
  Serial.print(millis());
  Serial.println(F(" ########################################"));
}


void printDHT(void)
{
  if(readDHT()) {
       // log DHT data
       Serial.print(F("DHT,"));
       Serial.print(dhtData.humidity);
       Serial.print(F(","));
       Serial.print(dhtData.temp_c);
       Serial.print(F(","));
       Serial.print(dhtData.temp_f);
       Serial.print(F(","));
       Serial.println(dhtData.heatIndex);
       // logfile.flush();
    }
    else {
       Serial.println(F("DHT,-1,-1,-1,-1"));
       // logfile.flush();
    }
}

void logDHT(void)
{
  if(readDHT()) {
       // log DHT data
       logfile.print(F("DHT,"));
       logfile.print(dhtData.humidity);
       logfile.print(F(","));
       logfile.print(dhtData.temp_c);
       logfile.print(F(","));
       logfile.print(dhtData.temp_f);
       logfile.print(F(","));
       logfile.println(dhtData.heatIndex);
       // logfile.flush();
    }
    else {
       logfile.println(F("DHT,-1,-1,-1,-1"));
       // logfile.flush();
    }
}

// poll the DHT every other cycle (0.5Hz) this relies for the GPS to be at the 1Hz Rate
boolean readDHT() {
  if(dhtSkip) {
     Serial.println(F("DHT: too early "));
     dhtData.valid = false;
     dhtSkip = !dhtSkip;
     return false;
  }

  dhtSkip = !dhtSkip;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();           // Read temperature as Celsius
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);    // Read temperature as Fahrenheit

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("DHT: Failed to read from sensor!"));
    return false;
  }

  float hi = dht.computeHeatIndex(f, h);      // Compute heat index Must send in temp in Fahrenheit!

  // GK TODO: save values to struct
  dhtData.humidity = h;
  dhtData.temp_c = t;
  dhtData.temp_f = f;
  dhtData.heatIndex = hi;
  dhtData.valid = true;

  Serial.print(F("DHT: "));
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("% "));
  Serial.print(F("Temp: "));
  Serial.print(t);
  Serial.print(F("*C "));
  Serial.print(f);
  Serial.print(F(" *F "));
  Serial.print(F("HI: ")); // heat index
  Serial.print(hi);
  Serial.println(F(" *F"));

  return true;
}

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}


// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
    Serial.print("error=");
    Serial.println(errno);
  } // while()
}
