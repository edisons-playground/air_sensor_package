#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>

File logfile;
#define chipSelect 10
#define ledPin 3  

// K30 Init
byte read_CO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2 (see app note)
byte response[] = {0,0,0,0,0,0,0};                             //create an array to store the response
int valMultiplier = 1;                                         //multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
SoftwareSerial K_30_Serial(4,5);                               //Sets up a virtual serial port, Using pin 4 for Rx and pin 5 for Tx 


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
 
  Serial.println(F("Initializing DK30 sensor..."));
  K_30_Serial.begin(9600);
  Serial.println(F("  K30 initialized"));

}


void loop() {
    logK30();

    logfile.flush();
}


void K30sendRequest(byte packet[]) {
  while(!K_30_Serial.available()) { //keep sending request until we start to get a response
    K_30_Serial.write(read_CO2,7);
    //Serial.println(F("writing..."));
    delay(50);
  }
  int timeout=0;  //set a timeoute counter
  while(K_30_Serial.available() < 7 ) { //Wait to get a 7 byte response
    timeout++;  
    if(timeout > 10) {   //if it takes to long there was probably an error
        while(K_30_Serial.available())  //flush whatever we have
          K_30_Serial.read();
          
          break;                        //exit and try again
      }
      delay(50);
  }
  for (int i=0; i < 7; i++) {
    response[i] = K_30_Serial.read();
  }  
}


unsigned long K30getValue(byte packet[]) {
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];                         //low byte for value is 5th byte in the packet
    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value
    return val* valMultiplier;
}



void logK30(void)
{
    K_30_Serial.listen();
    K30sendRequest(read_CO2);
    unsigned long valCO2 = K30getValue(response);
    Serial.print(F("K30: Co2 ppm = "));
    Serial.println(valCO2);
    logfile.print(F("K30,"));
    logfile.println(valCO2);
}
