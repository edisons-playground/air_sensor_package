#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include <Adafruit_TSL2561_U.h>


struct _dhtData {
  boolean valid;
  float   humidity;
  float   temp_c;
  float   temp_f;
  float   heatIndex;  
};

_dhtData dhtData;
File logfile;

byte read_CO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}; 
byte response[] = {0,0,0,0,0,0,0};                             
SoftwareSerial K_30_Serial(4,5);  

DHT dht(2, DHT22);

SoftwareSerial gps_Serial(8, 7);
Adafruit_GPS GPS(&gps_Serial);

// Light Init
Adafruit_TSL2561_Unified lght_snsr = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);        // The address will be different depending on whether you leave the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively                                    

void setup() {
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(10, OUTPUT);   
  
  sdCardInitialization();

  K_30_Serial.begin(9600);

  dht.begin();
 
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);      
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  GPS.sendCommand(PGCMD_ANTENNA);
  
  gps_Serial.println(PMTK_Q_RELEASE);
  
  if(!lght_snsr.begin())
  {
    Serial.print("No TSL2561 detected");
    while(1);
  } 
 
  configureLightSensor();
}

int loopCount = 0;
int dhtRead = true;

void loop() {
  gps_Serial.listen();
  char c = GPS.read();
  
  
  if (GPS.newNMEAreceived()) {
    
    if (!GPS.parse(GPS.lastNMEA()))  {
    return;
    }
    
    if (!GPS.fix) {
        Serial.println("No Fix");
        return;
    }

    
    logfile.println(F("#####"));
    printHeader();
    
    char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))  { 
      Serial.println(F("error with writing to SD")); 
    }
    
    logGPSData();
    logK30();
    logDHT();
    logLight();
    logfile.flush();
  }
}


void sdCardInitialization(void)
{
  if (!SD.begin(10)) {
    Serial.println(F("SD Card init. failed!"));
    while(1);
  } else {
  Serial.println(F("SD Card init successful!"));
  }
  
  char filename[15];
  sprintf(filename, "/LOG00.txt");
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
    return val* 1;
}

void logK30(void)
{
    K_30_Serial.listen();
    K30sendRequest(read_CO2);
    unsigned long valCO2 = K30getValue(response);
    logfile.print(F("K30,"));
    logfile.println(valCO2);
    logfile.flush();
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
       logfile.flush();
    }
//    else {
//       logfile.println(F("DHT,-1,-1,-1,-1"));
//       logfile.flush();
//    }
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


void configureLightSensor(void)         //Configures the gain and integration time for the TSL2561
{
  // lght_snsr.setGain(TSL2561_GAIN_1X);      // No gain ... use in bright light to avoid sensor saturation
  // lght_snsr.setGain(TSL2561_GAIN_16X);     // 16x gain ... use in low light to boost sensitivity
  lght_snsr.enableAutoRange(true);            // Auto-gain ... switches automatically between 1x and 16x
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  lght_snsr.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // light_snsr.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // light_snsr.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
 
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

    
void logLight(void)
{ 
  sensors_event_t event;      //Get a new sensor event
  lght_snsr.getEvent(&event);
 
  // Display the results (light is measured in lux)
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");
    logfile.print(F("Light Sensor, "));
    logfile.print(event.light);
    logfile.print(F(" lux"));
    logfile.println();
    } else {
      Serial.println("Sensor overload");      // If event.light = 0 lux the sensor is probably saturated and no reliable data could be generated!
      logfile.println(F("Light Sensor Overloaded"));
    }
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
