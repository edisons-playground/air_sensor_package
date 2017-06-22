#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"


struct _dhtData {
  boolean valid;
  float   humidity;
  float   temp_c;
  float   temp_f;
  float   heatIndex;
};

_dhtData dhtData;
File logfile;

#define chipSelect 10
#define ledPin 3
#define DHTTYPE DHT22
#define DHTPIN 2
#define GPSECHO  false
#define LOG_FIXONLY true

byte read_CO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
byte response[] = {0,0,0,0,0,0,0};
int valMultiplier = 1;
SoftwareSerial K_30_Serial(4,5);

DHT dht(DHTPIN, DHTTYPE);

SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);


void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);

  sdCardInitialization();

  K_30_Serial.begin(9600);

  dht.begin();

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);   // uncomment this line to turn on only the "minimum recommended" data; For logging data, we don't suggest using anything but either RMC only or RMC+GGA to keep the log files at a reasonable size
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  GPS.sendCommand(PGCMD_ANTENNA);

  gps_Serial.println(PMTK_Q_RELEASE);
}

int loopCount = 0;
int dhtRead = true;

void loop() {
  gps_Serial.listen();
  char c = GPS.read();

  if (GPSECHO) {
     if (c)   Serial.print(c);
  }


  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))  { // this also sets the newNMEAreceived() flag to false
      //Serial.println(F("Failed to parse GPS data"));
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    if (LOG_FIXONLY && !GPS.fix) {
        Serial.println("No Fix");
        return;
    }

    printHeader();

    char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))  {
      Serial.println(F("error with writing to SD"));
    }

    logData();
    logfile.flush();
  }
}


void sdCardInitialization(void)
{
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD Card init. failed!"));
    while(1);
  } else {
  Serial.println(F("SD Card init successful!"));
  }

  char filename[15];
  sprintf(filename, "/LOG00.csv");
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    if (! SD.exists(filename)) {
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

void logData() {
  logfile.print(F("\n"));
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
    logfile.print(F(","));
    logK30();
    logfile.print(F(","));
    logDHT();
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


void K30sendRequest(byte packet[]) {
  while(!K_30_Serial.available()) {
    K_30_Serial.write(read_CO2,7);
    delay(50);
  }
  int timeout=0;
  while(K_30_Serial.available() < 7 ) {
    timeout++;
    if(timeout > 10) {
        while(K_30_Serial.available())
          K_30_Serial.read();

          break;
      }
      delay(50);
  }
  for (int i=0; i < 7; i++) {
    response[i] = K_30_Serial.read();
  }
}


unsigned long K30getValue(byte packet[]) {
    int high = packet[3];
    int low = packet[4];
    unsigned long val = high*256 + low;
    return val* valMultiplier;
}

void logK30(void)
{
    K_30_Serial.listen();
    K30sendRequest(read_CO2);
    unsigned long valCO2 = K30getValue(response);
    logfile.print(valCO2);
    logfile.flush();
}


void logDHT(void)
{
  if(readDHT()) {
       // log DHT data
       logfile.print(dhtData.humidity);
       logfile.print(F(","));
       logfile.print(dhtData.temp_c);
       logfile.print(F(","));
       logfile.print(dhtData.temp_f);
       logfile.print(F(","));
       logfile.print(dhtData.heatIndex);
       logfile.flush();
    }
    else {
       logfile.println(F("-1,-1,-1,-1"));
       logfile.flush();
    }
}

boolean readDHT() {
  int loopFourth = loopCount % 4;
  if (loopFourth == 0) dhtRead = !dhtRead;

  if(!dhtRead) {
     dhtData.valid = false;
     return false;
  }

  dhtRead = !dhtRead;

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("DHT: Failed to read from sensor!"));
    return false;
  }

  float hi = dht.computeHeatIndex(f, h);

  dhtData.humidity = h;
  dhtData.temp_c = t;
  dhtData.temp_f = f;
  dhtData.heatIndex = hi;
  dhtData.valid = true;

  return true;
}

void printHeader() {
  Serial.print(F("##### Sample "));
  Serial.print(loopCount++);
  Serial.print(F(" "));
  Serial.print(millis());
  Serial.println(F(" ########################################"));
}


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
