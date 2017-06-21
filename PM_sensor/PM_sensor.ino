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
    logfile.print("concentration = ");
    logfile.print(pm25pcs2ugm3(concentration));
    logfile.println(" ugm/0.01cf");         //Check if correct unit
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}


float pm25pcs2ugm3 (float concentration_pcs)
{
  double pi = 3.14159;
  double density = 1.65 * pow (10, 12);
  double r25 = 0.44 * pow (10, -6);
  double vol25 = (4/3) * pi * pow (r25, 3);
  double mass25 = density * vol25;
  double K = 3531.5;

  return (concentration_pcs) * K * mass25;
}
