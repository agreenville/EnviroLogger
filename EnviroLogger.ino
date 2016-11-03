
// load libraries
#include <OneWire.h> // for DS18B20 temp sensor
#include "DHT.h" // DHT humidity and temp sensor
#include <Sensirion.h> // Soil temp and humidity
#include <SFE_BMP180.h> // Bosch BMP180 barometric pressure sensor.
#include <Wire.h> // Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"
//#include <SPI.h> // for data logger shield
#include <SD.h> //sd card library

// Real time clock function
RTC_PCF8523 rtc;

// DHT pins and function
const uint8_t DHTPIN = 5;     //  digital pin DHT connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// DS18S20 Signal pin on digital 9
int DS18B20_Pin = 9; 

// defining soil temp and humidity pins and variables
const uint8_t dataPin  =  12; // soil data
const uint8_t clockPin =  11; // soil clock
float SoilTemp; 
float SoilHumidity;
float SoilDew;
Sensirion tempSensor = Sensirion(dataPin, clockPin);

//Temperature chip i/o
OneWire ds(DS18B20_Pin);  // on digital pin 9

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// BMP180 barometric pressure sensor.
SFE_BMP180 pressure;
#define ALTITUDE 273 // define Altitude in meters

// logging
// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
int LOG_INTERVAL = 5000; // mills between entries (reduce to take more/faster data)

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void setup() {
  Serial.begin(9600);

// initialize the SD card
  Serial.print(F("Initializing SD card..."));
    
// see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
  }
  Serial.println(F("card initialized."));
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    Serial.println(F("couldnt create file"));
  }
  
  Serial.print(F("Logging to: "));
  Serial.println(filename);
  
  // real time clock check
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }
  // real time clock check for running and code to set time from computer
  if (! rtc.initialized()) {
    Serial.println(F("RTC is NOT running!"));
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //uncomment to set clock
  }

   // Initialize the pressure sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println(F("BMP180 init success"));
  else
  {
    Serial.println(F("BMP180 init fail\n\n"));
    while(1); // Pause forever.
  }
  
  dht.begin();

  // log field headings to sd card
   logfile.println(F("Date and Time, Humidity (%), Temperature (*C), DS18B20 sensor (*C), Heat index (*C), Soil Temperature (*C), Soil humidity (%), Soil dewpoint (*C), Absolute pressure (mb), Absolute pressure (inHg), Relative pressure (mb), Relative pressure (inHg)")); 
  logfile.flush();
  
// echo to serial
Serial.println(F("Date and Time, Humidity (%), Temperature (*C), DS18B20 sensor (*C), Heat index (*C), Soil Temperature (*C), Soil humidity (%), Soil dewpoint (*C), Absolute pressure (mb), Absolute pressure (inHg), Relative pressure (mb), Relative pressure (inHg)")); 
   
}

void loop() {

delay(LOG_INTERVAL); //log interval
 
  // DS18B20 sensor
  double temperature = getTemp(); //float

  //RTC
    DateTime now = rtc.now();
  
  // DHT Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  double h = dht.readHumidity(); //float
  // Read temperature as Celsius (the default)
  double t = dht.readTemperature(); //float
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  // Compute heat index in Celsius (isFahreheit = false)
  double hic = dht.computeHeatIndex(t, h, false); //float

  // soil temp and humidity
  tempSensor.measure(&SoilTemp, &SoilHumidity, &SoilDew );

  // SFE_BMP180 barometric pressure sensor start
   char status;
  double T,P,p0,a;
  // You must first get a temperature measurement to perform a pressure reading.
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
  }
    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {  
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
    
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

status = pressure.getPressure(P,T);
  p0 = pressure.sealevel(P,ALTITUDE);       
      }}
// End pressure

// logging
  //RTC
   logfile.print(now.year(), DEC);
    logfile.print(F("/"));
    if (now.month() < 10) { logfile.print(F("0")); }
    logfile.print(now.month(), DEC);
    logfile.print(F("/"));
    if (now.day() < 10) { logfile.print(F("0")); }
    logfile.print(now.day(), DEC);
    logfile.print(F(" "));
    logfile.print(now.hour(), DEC);
    logfile.print(F(":"));
    if (now.minute() < 10) { logfile.print(F("0")); }
    logfile.print(now.minute(), DEC);
    logfile.print(F(":"));
    if ( now.second() < 10 ) { logfile.print( F("0" )); }
    logfile.print(now.second(), DEC);
    logfile.print(F(", "));

//sensors    
  logfile.print(h);
  logfile.print(F(", "));
  logfile.print(t);
  logfile.print(F(", "));
  logfile.print(temperature);
  logfile.print(F(", "));
  logfile.print(hic);
  logfile.print(F(", "));
  logfile.print(SoilTemp);
  logfile.print(F(", "));
  logfile.print(SoilHumidity);
  logfile.print(F(", "));
  logfile.print(SoilDew);
  logfile.print(F(", "));
  logfile.print(P,2); // absolute pressure (mb)
  logfile.print(F(", "));
  logfile.print(P*0.0295333727,2); //absolute pressure (inHg)
  logfile.print(F(", "));
  logfile.print(p0,2); // relative pressure (mb)
  logfile.print(F(", "));
  logfile.println(p0*0.0295333727,2); // relative pressure (inHg)
 logfile.flush();
      
 // Printing to serial  
  //RTC
   Serial.print(now.year(), DEC);
    Serial.print(F("/"));
    if (now.month() < 10) { Serial.print(F("0")); }
    Serial.print(now.month(), DEC);
    Serial.print(F("/"));
    if (now.day() < 10) { Serial.print(F("0")); }
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(F(":"));
    if (now.minute() < 10) { Serial.print(F("0")); }
    Serial.print(now.minute(), DEC);
    Serial.print(F(":"));
    if (now.second() < 10) { Serial.print(F("0")); }
    Serial.print(now.second(), DEC);
    Serial.print(F(", "));
//sensors    
  Serial.print(h);
  Serial.print(F(", "));
  Serial.print(t);
  Serial.print(F(", "));
  Serial.print(temperature);
  Serial.print(F(", "));
  Serial.print(hic);
  Serial.print(F(", "));
  Serial.print(SoilTemp);
  Serial.print(F(", "));
  Serial.print(SoilHumidity);
  Serial.print(F(", "));
  Serial.print(SoilDew);
  Serial.print(F(", "));
  Serial.print(P,2); // absolute pressure (mb)
  Serial.print(F(", "));
  Serial.print(P*0.0295333727,2); //absolute pressure (inHg)
  Serial.print(F(", "));
  Serial.print(p0,2); // relative pressure (mb)
  Serial.print(F(", "));
  Serial.println(p0*0.0295333727,2); // relative pressure (inHg)


 
}

// user functions
// DS18B20 sensor

double getTemp() { //float
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -100;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println(F("CRC is not valid!"));
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print(F("Device is not recognized"));
    return -2000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  double tempRead = ((MSB << 8) | LSB); //using two's compliment //float
  double TemperatureSum = tempRead / 16;  //float

  return TemperatureSum;
  // End DS18B20 sensor function

}

