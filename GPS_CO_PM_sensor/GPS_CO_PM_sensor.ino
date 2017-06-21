#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <math.h>

File logfile;
int const smallPM1 = 9;       // Pin # of Data line
long const sampleRate = 20;
long measurementCount = 0;
long smallPM1Count = 0;
long priorSampleTime = 0;
double smallPM1percentRunning;

unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

#define chipSelect 10
#define ledPin 3             
#define GPSECHO  false       // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console, Set to 'true' if you want to debug and listen to the raw GPS sentences
#define LOG_FIXONLY true     //set to true to only log to SD when GPS has a fix, for debugging, keep it false

// GPS Init
SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);
                                    

void setup() {
  Serial.begin(115200);
  pinMode (smallPM1, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);   
  
  sdCardInitialization();

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
  starttime = millis();//get the current time;
}

int loopCount = 1;

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
    logCO();
    logfile.print(F(","));
    logPM();
  }
}

float gpsConverter(float gps)
{
  int gps_min = (int)(gps/100);
  float gps_sec = fmod(gps, 100) / 60;
  float gps_dec = gps_min + gps_sec;
  return gps_dec;
}

void logPM(void)
{
  duration = pulseIn(smallPM1, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;

  if ((millis()-starttime) >= sampletime_ms)//if the sampel time = = 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=&gt;100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    logfile.print("concentration = ");
    logfile.print(concentration);
    logfile.println(" pcs/0.01cf");
    Serial.print("concentration = ");
    Serial.print(concentration);
    Serial.println(" pcs/0.01cf");
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}


void logCO(void)
{
  int reading = analogRead(A0);
  Serial.print("CO Sensor, "); Serial.println(reading);
  logfile.print(F("CO Sensor "));
  logfile.print(reading);
  analogWrite(ledPin, reading);
//  logfile.println();
  delay(10);
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
