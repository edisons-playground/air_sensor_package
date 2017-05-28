#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
 
File logfile;  

#define chipSelect 10
#define ledPin 3 

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(10, OUTPUT);
  
  // SD Card Init
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD Card init. failed!"));
    while(1);
  } else {
  Serial.println(F("SD Card init successful!"));
  }
  
  // File Init
  char filename[15];
  sprintf(filename, "/LOG27.txt");           //manually input file name
//  for (uint8_t i = 0; i < 100; i++) {      //uncomment to load first file
//    filename[4] = '0' + i/10;
//    filename[5] = '0' + i%10;
//    if (! SD.exists(filename)) {       
//      break;
//    }
//  }
  
  Serial.print(F("reading file: "));
  Serial.println(filename);
  logfile = SD.open(filename, FILE_READ);
  if(logfile) {
    Serial.print(logfile.read());
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
