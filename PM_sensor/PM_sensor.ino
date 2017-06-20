#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>

int const smallPM1 = 3;       // Pin # of SmallPM data line
long const sampleRate = 20;
long measurementCount = 0;
long smallPM1Count = 0;
long priorSampleTime = 0;
double smallPM1percentRunning;
File logfile;

unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

#define chipSelect 10
#define ledPin 3

void setup() {
  Serial.begin(115200);
  pinMode (smallPM1, INPUT);

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

void loop() {

  // samplePMDetectors();         //sample particulate detectors for 2 seconds and update running Averages
  logPM();
  printPM();
  // timestampSerial();            //print time
  // printRunningPMDataToSerial(); //print percentages to Serial
  // logRunningPMDataToSerial();
  Serial.println();
  Serial.println();
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
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}

void printPM(void)
{
  duration = pulseIn(smallPM1, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;

  if ((millis()-starttime) >= sampletime_ms)//if the sampel time = = 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=&gt;100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.print("concentration = ");
    Serial.print(concentration);
    Serial.println(" pcs/0.01cf");
    lowpulseoccupancy = 0;
    starttime = millis();
  }
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
  }
  //calculate running PM percentages
  smallPM1percentRunning = 100.0 * smallPM1Count / measurementCount;
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
  Serial.println();
}

void logRunningPMDataToSerial() {
  logfile.println("Particulate Matter Data");
  logfile.print("Measurement Count:  ");
  logfile.println(measurementCount);
  logfile.print("Small PM detector 1: ");
  logfile.println(smallPM1percentRunning);
  logfile.println();
}
