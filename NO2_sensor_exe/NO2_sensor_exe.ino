#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>


File logfile;
#define chipSelect 10
#define ledPin 3
#define GPSECHO  false       
#define LOG_FIXONLY true     

// GPS Init
SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);

int const NO2port = A2;
float const NO2seriesResistor = 22000;
float NO2resistance;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode (NO2port, INPUT);  
  
  sdCardInitialization();
  
  Serial.println(F("Initializing GPS..."));
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // Set the update rate; 1 or 5 Hz update rate\
  GPS.sendCommand(PGCMD_ANTENNA);
  gps_Serial.println(PMTK_Q_RELEASE);

  Serial.println(F("Ready!"));
}

int loopCount = 1;

void loop(void) 
{ 
  gps_Serial.listen();
  char c = GPS.read();

  if (GPSECHO) {
     if (c)   Serial.print(c);
  }

  if (GPS.newNMEAreceived()) {
    GPS.lastNMEA();

    if (!GPS.parse(GPS.lastNMEA()))  { 
      return;  
    }

    if (LOG_FIXONLY && !GPS.fix) {
        Serial.println("No Fix");
        return;
    }

    logfile.println(F("\n#####"));  // new data marker
    printHeader();

    char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))  {  //write the string to the SD file
      Serial.println(F("error with writing to SD"));
    }
    
    logData();
    
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
  sprintf(filename, "/LOG00.csv");
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


void logNO2() {
  //read gas sensor data
  int NO2rawInput = analogRead(NO2port);
  //calculate resistances
  NO2resistance = NO2seriesResistor * ((1023.0 / NO2rawInput) - 1.0);
  logfile.print(NO2resistance);
  analogWrite(ledPin, NO2resistance);
  delay(10);
}

void logData() {
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
    logNO2();
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
