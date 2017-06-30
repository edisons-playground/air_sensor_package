#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>


File logfile;
#define chipSelect 10
#define ledPin 3

int const O3port = A3;
float const O3seriesResistor = 22000;
float O3resistance;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode (O3port, INPUT);  
  
  sdCardInitialization();
}

void loop(void) 
{ 
    logO3();
    
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


void logO3() {
  //read gas sensor data
  int O3rawInput = analogRead(O3port);
  //calculate resistances
  O3resistance = O3seriesResistor * ((1023.0 / O3rawInput) - 1.0);
  Serial.println("O3 Resistance = "); Serial.println(O3resistance);
  logfile.print(F("O3 Sensor, "));
  logfile.print(O3resistance);
  analogWrite(ledPin, O3resistance);
  logfile.println();
  delay(10);
}

