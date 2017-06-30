#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>


File logfile;
#define chipSelect 10
#define ledPin 3

int const NO2port = A2;
float const NO2seriesResistor = 22000;
float NO2resistance;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode (NO2port, INPUT);  
  
  sdCardInitialization();
  
}

void loop(void) 
{ 
    logNO2();
    
    logfile.flush();
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
  Serial.println("NO2 Resistance = "); Serial.println(NO2resistance);
  logfile.print(F("NO2 Sensor, "));
  logfile.print(NO2resistance);
  analogWrite(ledPin, NO2resistance);
  logfile.println();
  delay(10);
}
