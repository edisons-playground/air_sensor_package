#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
  

File logfile;

#define chipSelect 10
#define ledPin 3             

// Light Init
Adafruit_TSL2561_Unified lght_snsr = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);        // The address will be different depending on whether you leave the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively


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
  
  Serial.println(F("Initializing Light sensor..."));
  if(!lght_snsr.begin())
  {
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } else {
    Serial.println(F("  Light initialized"));
  }
 
  displayLightSensorDetails();
  configureLightSensor();
  
}


void loop(void) 
{
    logLight();
    logfile.flush();
}


    
void displayLightSensorDetails(void)
{
  sensor_t sensor;
  lght_snsr.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
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

