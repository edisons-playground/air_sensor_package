#include <Time.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
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

#define DHTTYPE DHT22        // DHT 22  (AM2302)
#define DHTPIN 2             // pin for DHT communication
#define chipSelect 10
#define ledPin 3             
#define GPSECHO  false       // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console, Set to 'true' if you want to debug and listen to the raw GPS sentences
#define LOG_FIXONLY true     //set to true to only log to SD when GPS has a fix, for debugging, keep it false

// K30 Init
byte read_CO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};                             //create an array to store the response
int valMultiplier = 1;                                         //multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
SoftwareSerial K_30_Serial(4,5);                               //Sets up a virtual serial port, Using pin 4 for Rx and pin 5 for Tx 
  
// GPS Init
SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);
                                    
// DHT Init
DHT dht(DHTPIN, DHTTYPE);


void setup() {
  Serial.begin(115200);
  Serial.println(F("\r\nAir Quality Sensing Package Initializing"));
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);    // make sure that the default chip select pin is set to output, even if you don't use it:
  
  sdCardInitialization();

  Serial.println(F("Initializing DHT sensor..."));
  dht.begin();
  Serial.println(F("  DHT initialized"));

  Serial.println(F("Initializing DK30 sensor..."));
  K_30_Serial.begin(9600);
  Serial.println(F("  K30 initialized"));

  Serial.println(F("Initializing GPS..."));
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);   // uncomment this line to turn on only the "minimum recommended" data; For logging data, we don't suggest using anything but either RMC only or RMC+GGA to keep the log files at a reasonable size
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // Set the update rate; 1 or 5 Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  // GPS.sendCommand(PGCMD_NOANTENNA);
  GPS.sendCommand(PGCMD_ANTENNA);
  
  dhtData.valid = false;
  
  // Ask for firmware version
  gps_Serial.println(PMTK_Q_RELEASE);
  
  Serial.println(F("Ready!"));
}

int lastMillis = 0;
int loopCount = 1;
//int lastDHTpoll = 0;  // last millis when DHT was polled
//int dhtPollThreshold = 1900;
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

    logDHT();
    
    logK30();

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


void printHeader() {
  Serial.print(F("##### Sample "));   // new data marker
  Serial.print(loopCount++);
  Serial.print(F(" "));
  Serial.print(millis());
  Serial.println(F(" ########################################")); 
}


void K30sendRequest(byte packet[]) {
  while(!K_30_Serial.available()) { //keep sending request until we start to get a response
    K_30_Serial.write(read_CO2,7);
    //Serial.println(F("writing..."));
    delay(50);
  }
  int timeout=0;  //set a timeoute counter
  while(K_30_Serial.available() < 7 ) { //Wait to get a 7 byte response
    timeout++;  
    if(timeout > 10) {   //if it takes to long there was probably an error
        while(K_30_Serial.available())  //flush whatever we have
          K_30_Serial.read();
          
          break;                        //exit and try again
      }
      delay(50);
  }
  for (int i=0; i < 7; i++) {
    response[i] = K_30_Serial.read();
  }  
}


unsigned long K30getValue(byte packet[]) {
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];                         //low byte for value is 5th byte in the packet
    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value
    return val* valMultiplier;
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


void logK30(void)
{
    K_30_Serial.listen();
    K30sendRequest(read_CO2);
    unsigned long valCO2 = K30getValue(response);
    Serial.print(F("K30: Co2 ppm = "));
    Serial.println(valCO2);
    logfile.print(F("K30,"));
    logfile.println(valCO2);
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
