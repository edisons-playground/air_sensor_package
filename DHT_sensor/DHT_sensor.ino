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

// DHT Init
DHT dht(DHTPIN, DHTTYPE);


void setup() {
  Serial.begin(115200);
  
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
}

int dhtSkip = false;

void loop() {
  logDHT();
  Serial.println();
  Serial.println();

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
